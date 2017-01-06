/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#include <ros/ros.h>

#include <srslib_framework/io/IO.hpp>
#include <srslib_framework/utils/Logging.hpp>

#include <BrainStemMessages.hpp>
#include <BrainStemMessageProcessor.hpp>

#include <hw_message/LogHandler.hpp>
#include <hw_message/HardwareInfoHandler.hpp>
#include <hw_message/OperationalStateHandler.hpp>
#include <hw_message/PowerStateHandler.hpp>
#include <hw_message/OdometryRpmHandler.hpp>
#include <hw_message/OdometryPoseHandler.hpp>
#include <hw_message/ButtonPressedHandler.hpp>

#include <sw_message/ResetHandler.hpp>
#include <sw_message/PingHandler.hpp>
#include <sw_message/SetMotionStateHandler.hpp>
#include <sw_message/SetOdometryRpmHandler.hpp>
#include <sw_message/SetVelocityHandler.hpp>
#include <sw_message/ShutdownHandler.hpp>
#include <sw_message/SoundHandler.hpp>
#include <sw_message/UpdateUIHandler.hpp>



namespace srs {

using namespace ros;

BrainStemMessageProcessor::BrainStemMessageProcessor(std::shared_ptr<IO> pIO) :
	io_(pIO),
	setupComplete_(false),
	syncState_(false),
	sentPing_(false),
	isConnected_(false),
	hasValidHardareInfo_(false),
	lastHardareInfoRequestTime_(),
	hasValidOperationalState_(false),
	lastOperationalStateRequestTime_(),
	lastMessageTime_(),
	connectedChannel_(),
	odometryRpmChannel_(),
	odometryPoseChannel_(),
	hardwareInfoChannel_(),
	operationalStateChannel_(),
	powerStateChannel_()
{
    hardwareInfoHandler_.reset(new HardwareInfoHandler(hardwareInfoChannel_));
    operationalStateHandler_.reset(new OperationalStateHandler(operationalStateChannel_));

    addHardwareMessageHandler(hardwareInfoHandler_);
    addHardwareMessageHandler(operationalStateHandler_);
    addHardwareMessageHandler(HardwareMessageHandlerPtr(new LogHandler()));
    addHardwareMessageHandler(HardwareMessageHandlerPtr(new PowerStateHandler(powerStateChannel_)));
    addHardwareMessageHandler(HardwareMessageHandlerPtr(new OdometryRpmHandler(odometryRpmChannel_)));
    addHardwareMessageHandler(HardwareMessageHandlerPtr(new OdometryPoseHandler(odometryPoseChannel_)));
    addHardwareMessageHandler(HardwareMessageHandlerPtr(new ButtonPressedHandler(buttonPressedChannel_)));

    addSoftwareMessage(SoftwareMessagePtr(new ResetHandler(this)));
    addSoftwareMessage(SoftwareMessagePtr(new PingHandler(this)));
    addSoftwareMessage(SoftwareMessagePtr(new SetMotionStateHandler(this)));
    addSoftwareMessage(SoftwareMessagePtr(new SetOdometryRpmHandler(this)));
    addSoftwareMessage(SoftwareMessagePtr(new SetVelocityHandler(this)));
    addSoftwareMessage(SoftwareMessagePtr(new ShutdownHandler(this)));
    addSoftwareMessage(SoftwareMessagePtr(new SoundHandler(this)));
    addSoftwareMessage(SoftwareMessagePtr(new UpdateUIHandler(this)));

    for (auto handler : softwareHandlers_)
    {
    	handler->attach();
    }
}

BrainStemMessageProcessor::~BrainStemMessageProcessor()
{

}

//////////////////////////////////////////////////////////////////////////
// Message Processing
//////////////////////////////////////////////////////////////////////////

void BrainStemMessageProcessor::processHardwareMessage(vector<char> buffer)
{
    BRAIN_STEM_MSG eCommand = BRAIN_STEM_MSG::UNKNOWN;

    if (buffer.size() > 0)
    {
        eCommand = static_cast<BRAIN_STEM_MSG>(buffer[0]);

		ros::Time currentTime = ros::Time::now();

		if (setupComplete_ ||
			eCommand == BRAIN_STEM_MSG::OPERATIONAL_STATE ||
			eCommand == BRAIN_STEM_MSG::HARDWARE_INFO)
		{
			// Go through the registered message handlers and communicate the data if the key matches
			HwMessageHandlerMapType::iterator handler = hwMessageHandlers_.find(eCommand);
			if (handler != hwMessageHandlers_.end())
			{
				try
				{
					handler->second->receiveData(currentTime, buffer);

				}
				catch(std::runtime_error& error)
				{
					// If it arrives here, the message failed to parse
					ROS_ERROR_STREAM("Brainstem driver: Message from brainstem malformed: " <<
						static_cast<char>(eCommand) << ", data: " << ToHex(buffer) << ", exception: " << error.what());
				}
			}
			else
			{
				if (isSetupComplete() &&
					eCommand != BRAIN_STEM_MSG::SYSTEM_VOLTAGE  &&
					eCommand != BRAIN_STEM_MSG::SENSOR_FRAME &&
					eCommand != BRAIN_STEM_MSG::MESSAGE)
				{
					ROS_ERROR_STREAM("Brainstem driver: Unknown message from brainstem: " <<
						static_cast<int>(eCommand) << ", data: " << ToHex(buffer));
				}
			}
		}

		checkSetupComplete();

		lastMessageTime_ = currentTime;
    }
}

void BrainStemMessageProcessor::getOperationalState()
{
	if (sentPing_)
	{
		ROS_DEBUG("Brainstem driver: GetOperationalState");

		CommandData msg = { static_cast<uint8_t>(BRAIN_STEM_CMD::GET_OPERATIONAL_STATE) };

		// Get the operational state
		writeToSerialPort(reinterpret_cast<char*>(&msg), sizeof(msg));
	}
}

void BrainStemMessageProcessor::getHardwareInformation()
{
	if (sentPing_)
	{
		ROS_DEBUG("Brainstem driver: GetHardwareInformation");

		CommandData msg = { static_cast<uint8_t>(BRAIN_STEM_CMD::GET_HARDWARE_INFO) };

		// Get the hardware information (version, configuration, etc.)
		writeToSerialPort(reinterpret_cast<char*>(&msg), sizeof(msg));
	}
}

void BrainStemMessageProcessor::sendDimensions()
{
	for (auto dimension : dimensions_)
	{
		setDimension(dimension.first, dimension.second);
	}
}

void BrainStemMessageProcessor::shutdown()
{
	ROS_INFO("Brainstem driver: Shutdown");

	uint8_t cMessage = static_cast<uint8_t>(BRAIN_STEM_CMD::SHUTDOWN);

	writeToSerialPort(reinterpret_cast<char*>(&cMessage), 1);
}

//////////////////////////////////////////////////////////////////////////
// Bridge Callbacks
//////////////////////////////////////////////////////////////////////////

void BrainStemMessageProcessor::setConnected(bool isConnected)
{
	ROS_INFO("Brainstem driver: setConnected: %d", isConnected);

	if (isConnected != isConnected_)
	{
		bool resync = isSetupComplete();

		isConnected_ = isConnected;

		if (isConnected)
		{
			checkSetupComplete();
		}
		else
		{
			ROS_INFO("Brainstem driver: hid disconnected (resetting state).");

			// Reset state so we sync again when the brainstem reconnects
			sentPing_ = false;

			setupComplete_ = false;

			hardwareInfoHandler_->reset();

			operationalStateHandler_->reset();
		}
	}
}

void BrainStemMessageProcessor::getHardwareInfo(const ros::Time& now)
{
	if (!hardwareInfoHandler_->hasValidMessage())
	{
		ros::Duration duration = now - lastHardareInfoRequestTime_;
		if (duration.toSec() > 1.0f)
		{
			// Send another request
			getHardwareInformation();

			lastHardareInfoRequestTime_ = now;
		}
	}
}

void BrainStemMessageProcessor::getOperationalState(const ros::Time& now)
{
	if (!operationalStateHandler_->hasValidMessage())
	{
		ros::Duration duration = now - lastOperationalStateRequestTime_;
		if (duration.toSec() > 1.0f)
		{
			// Send another request
			getOperationalState();

			lastOperationalStateRequestTime_ = now;
		}
	}
}


void BrainStemMessageProcessor::checkForBrainstemFaultTimer(const ros::TimerEvent& event)
{
	bool brainstemTimeout = false;

	if (isSetupComplete())
	{
		if ((event.current_expected - lastMessageTime_).toSec() > FAULT_TIMEOUT)
		{
			brainstemTimeout = true;
		}
	}

	operationalStateHandler_->setBrainstemTimeout(brainstemTimeout);
}

void BrainStemMessageProcessor::addHardwareMessageHandler(HardwareMessageHandlerPtr hardwareMessageHandler)
{
    hwMessageHandlers_[hardwareMessageHandler->getKey()] = hardwareMessageHandler;

	hardwareHandlers_.push_back(hardwareMessageHandler);
}

void BrainStemMessageProcessor::addSoftwareMessage(SoftwareMessagePtr softwareMessage)
{
	softwareHandlers_.push_back(softwareMessage);
}

bool BrainStemMessageProcessor::isSetupComplete() const
{
	return setupComplete_;
}

void BrainStemMessageProcessor::checkSetupComplete()
{
	if (!setupComplete_)
	{
		if (hardwareInfoHandler_->hasValidMessage() &&
			operationalStateHandler_->hasValidMessage() &&
			isConnected_)
		{
			bool resync = syncState_;

			ROS_INFO("Brainstem driver: Setup complete (received hardware info and operational state messages.");

			io_->setSynced(true);

			setupComplete_ = true;

			syncState_ = true;

			sendDimensions();

			// If this is a reconnect sync the brainstem statate
			if (resync)
			{
				ROS_INFO("Brainstem driver: Resyncing brainstem state");

				for (auto handler : softwareHandlers_)
				{
					handler->sync();
				}
			}

			connectedChannel_.publish(syncState_);
		}
		else
		{
			if (sentPing_)
			{
				ros::Time currentTime = ros::Time::now();

				getHardwareInfo(currentTime);

				getOperationalState(currentTime);
			}
		}
	}
}

void BrainStemMessageProcessor::setDimension(DIMENSION dimension, float value)
{
	if (io_->isOpen())
	{
		static std::map<DIMENSION, std::string> mapDimensionName;

		if (!mapDimensionName.size())
		{
			mapDimensionName[DIMENSION::WHEEL_BASE_LENGTH] = "wheel_base_length";
			mapDimensionName[DIMENSION::LEFT_WHEEL_RADIUS] = "left_wheel_radius";
			mapDimensionName[DIMENSION::RIGHT_WHEEL_RADIUS] = "right_wheel_radius";
		};

		DimensionData msg = {
			static_cast<uint8_t>(BRAIN_STEM_CMD::SET_DIMENSION),
			static_cast<uint8_t>(dimension),
			static_cast<float>(value)
		};

		ROS_DEBUG("Brainstem driver: Setting dimension: %s => %f",
			mapDimensionName[dimension].c_str(), value);

		// Get the hardware information (version, configuration, etc.)
		writeToSerialPort(reinterpret_cast<char*>(&msg), sizeof(msg));
	}
	else
	{
		dimensions_[dimension] = value;
	}
}

void BrainStemMessageProcessor::sendCommand(char* command, std::size_t size)
{
	if (size)
	{
		writeToSerialPort(command, size);
	}
}

void BrainStemMessageProcessor::ping()
{
	if (!sentPing_)
	{
		sentPing_ = true;

		ros::Time currentTime = ros::Time::now();

		// Don't ask for the hardware information and the op state until we send the first ping
		// to avoid the brainstem timeouts propogating to the rest of the stack prematurely
		getHardwareInfo(currentTime);

		getOperationalState(currentTime);
	}
}

//////////////////////////////////////////////////////////////////////////
// Helper Methods
//////////////////////////////////////////////////////////////////////////

void BrainStemMessageProcessor::writeToSerialPort(char* pszData, std::size_t dwSize)
{
	if(io_->isOpen())
	{
		io_->write(std::vector<char>(pszData, pszData + dwSize));
	}
	else
	{
		ROS_ERROR_THROTTLE(60, "Brainstem driver: Attempt to write to the brain stem, but the serial port is not open!");
	}
}

} /* namespace srs */
