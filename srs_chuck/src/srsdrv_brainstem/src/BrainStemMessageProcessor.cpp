/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#include <ros/ros.h>

#include <srslib_framework/io/IO.hpp>
#include <srslib_framework/utils/Logging.hpp>
#include <srslib_framework/chuck/ChuckLimits.hpp>

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

#include <command/GetHardwareInfo.hpp>
#include <command/GetOperationalState.hpp>
#include <command/SetPhysicalDimension.hpp>
#include <command/SetMaxAllowedVelocity.hpp>

namespace srs {

using namespace ros;

BrainStemMessageProcessor::BrainStemMessageProcessor() :
	io_(),
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
	odometryPoseBrainstemChannel_(),
	hardwareInfoChannel_(),
	operationalStateChannel_(),
	powerStateChannel_(),
    hardwareInfoHandler_(new HardwareInfoHandler(hardwareInfoChannel_)),
    operationalStateHandler_(new OperationalStateHandler(operationalStateChannel_)),
	useBrainstemOdom_(false),
	odometryRpmHardwareHandler_(),
	odometryPoseHardwarewHandler_(),
	velocitySoftwareHandler_(),
	odometryRpmSoftwareHandler_()
{
    addHardwareMessageHandler(hardwareInfoHandler_);
    addHardwareMessageHandler(operationalStateHandler_);
    addHardwareMessageHandler(HardwareMessageHandlerPtr(new LogHandler()));
    addHardwareMessageHandler(HardwareMessageHandlerPtr(new PowerStateHandler(powerStateChannel_)));
    addHardwareMessageHandler(HardwareMessageHandlerPtr(new ButtonPressedHandler(buttonPressedChannel_)));

    addSoftwareMessage(SoftwareMessagePtr(new ResetHandler(this)));
    addSoftwareMessage(SoftwareMessagePtr(new PingHandler(this)));
    addSoftwareMessage(SoftwareMessagePtr(new SetMotionStateHandler(this)));
    addSoftwareMessage(SoftwareMessagePtr(new SetVelocityHandler(this)));
    addSoftwareMessage(SoftwareMessagePtr(new ShutdownHandler(this)));
    addSoftwareMessage(SoftwareMessagePtr(new SoundHandler(this)));
    addSoftwareMessage(SoftwareMessagePtr(new UpdateUIHandler(this)));
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

		if (isSetupComplete() ||
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
        GetOperationalState::send(this);
	}
}

void BrainStemMessageProcessor::getHardwareInformation()
{
    if (sentPing_)
    {
        GetHardwareInfo::send(this);
    }
}

void BrainStemMessageProcessor::sendDimensions()
{
	for (auto dimension : dimensions_)
	{
		setDimension(dimension.first, dimension.second);
	}
}

void BrainStemMessageProcessor::sendMaxAllowedVelocity()
{
    // Allow the hardware not to move faster than the specified velocity
    SetMaxAllowedVelocity::send(this,
        ChuckLimits::PHYSICAL_MAX_LINEAR,
        ChuckLimits::PHYSICAL_MAX_ANGULAR);
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
			sendMaxAllowedVelocity();
		}
		else
		{
			ROS_INFO("Brainstem driver: hid disconnected (resetting state).");

			// Reset state so we sync again when the brainstem reconnects
			sentPing_ = false;

			setupComplete_ = false;

			// brainstem disconnect occurs, handle pose reset
			if(odometryPoseHardwarewHandler_)
			{
				odometryPoseHardwarewHandler_->handlePoseReset();
			}

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

	hardwareHandlers_.insert(hardwareMessageHandler);
}

void BrainStemMessageProcessor::removeHardwareMessageHandler(HardwareMessageHandlerPtr hardwareMessageHandler)
{
	if (hardwareMessageHandler)
	{
		hardwareHandlers_.erase(hardwareMessageHandler);
	}
}

void BrainStemMessageProcessor::addSoftwareMessage(SoftwareMessagePtr softwareMessage)
{
	softwareHandlers_.insert(softwareMessage);

	softwareMessage->attach();
}

void BrainStemMessageProcessor::removeSoftwareMessage(SoftwareMessagePtr softwareMessage)
{
	if (softwareMessage)
	{
		softwareHandlers_.erase(softwareMessage);
	}
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
			sendMaxAllowedVelocity();

			// If this is a reconnect sync the brainstem state
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

void BrainStemMessageProcessor::setUseBrainstemOdom(bool useBrainstemOdom)
{
	bool hasHandlers = odometryRpmSoftwareHandler_ || velocitySoftwareHandler_;

	if (!hasHandlers || useBrainstemOdom_ != useBrainstemOdom)
	{
		if (useBrainstemOdom)
		{
			ROS_DEBUG("Brainstem driver: Using brainstem odometry");

			// This removeHardwareMessageHandler is only because we toggle main and alt topics for pose (for comparison in development)
			// TODO: remove after RPM is deprecated
			removeHardwareMessageHandler(odometryPoseHardwarewHandler_);

			// Brainstem odometry handlers
			odometryPoseHardwarewHandler_.reset(new OdometryPoseHandler(odometryPoseChannel_, true));
			addHardwareMessageHandler(odometryPoseHardwarewHandler_);

			velocitySoftwareHandler_.reset(new SetVelocityHandler(this));
			addSoftwareMessage(velocitySoftwareHandler_);

			// RPM odometry handlers
			removeHardwareMessageHandler(odometryRpmHardwareHandler_);
			odometryRpmHardwareHandler_ = nullptr;

			removeSoftwareMessage(odometryRpmSoftwareHandler_);
			odometryRpmSoftwareHandler_ = nullptr;
		}
		else
		{
			ROS_DEBUG("Brainstem driver: Using rpm odometry");

			// This removeHardwareMessageHandler is only because we toggle main and alt topics for pose (for comparison in development)
			// TODO: remove after RPM is deprecated
			removeHardwareMessageHandler(odometryPoseHardwarewHandler_);

			// RPM odometry handlers
			odometryRpmHardwareHandler_.reset(new OdometryRpmHandler(odometryRpmChannel_));
			addHardwareMessageHandler(odometryRpmHardwareHandler_);

			odometryRpmSoftwareHandler_.reset(new SetOdometryRpmHandler(this));
			addSoftwareMessage(odometryRpmSoftwareHandler_);

			// TODO: remove after RPM is deprecated
			odometryPoseHardwarewHandler_.reset(new OdometryPoseHandler(odometryPoseBrainstemChannel_, false));
			addHardwareMessageHandler(odometryPoseHardwarewHandler_);

			// Brainstem odometry handlers
			removeSoftwareMessage(velocitySoftwareHandler_);
			velocitySoftwareHandler_ = nullptr;
		}

		useBrainstemOdom_ = useBrainstemOdom;
	}
}

void BrainStemMessageProcessor::setDimension(SetPhysicalDimension::DimensionEnum dimension, float value)
{
	if (io_->isOpen())
	{
		SetPhysicalDimension::send(this, dimension, value);
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
