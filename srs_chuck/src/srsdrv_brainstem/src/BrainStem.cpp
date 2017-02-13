/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#include <BrainStem.hpp>

#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <srslib_framework/MsgHardwareInfo.h>
#include <srslib_framework/MsgOperationalState.h>
#include <srslib_framework/io/SerialIO.hpp>
#include <srslib_framework/io/HidIO.hpp>
#include <srslib_framework/platform/Thread.hpp>
#include <bitset>

#include <srslib_framework/math/BasicMath.hpp>
#include <srslib_framework/chuck/ChuckTopics.hpp>

#include <command/SetPhysicalDimension.hpp>

namespace srs
{

BrainStem::BrainStem(string name, int argc, char** argv) :
	RosUnit(name, argc, argv, REFRESH_RATE_HZ),
	messageProcessor_(),
	brainstemFaultTimer_(),
	nodeHandle_("~"),
	useEmulator_(false)
{
	nodeHandle_.param("use_emulator", useEmulator_, useEmulator_);

	connectionChanged( false );

	// Register dynamic configuration callback
	configServer_.setCallback(boost::bind(&BrainStem::cfgCallback, this, _1, _2));

	char* pszLeftWheelRadius = getenv("ROBOT_LEFT_WHEEL_RADIUS");
	char* pszRightWheelRadius = getenv("ROBOT_RIGHT_WHEEL_RADIUS");
	char* pszWheelBaseLength = getenv("ROBOT_WHEEL_BASE");

	if( pszLeftWheelRadius )
	{
		double leftWheelRadius = 0.0f;
		sscanf( pszLeftWheelRadius, "%lf", &leftWheelRadius );

		messageProcessor_.setDimension(SetPhysicalDimension::DimensionEnum::LEFT_WHEEL_RADIUS,
			leftWheelRadius);
	}

	if( pszRightWheelRadius )
	{
		double rightWheelRadius = 0.0f;
		sscanf( pszRightWheelRadius, "%lf", &rightWheelRadius );

		messageProcessor_.setDimension(SetPhysicalDimension::DimensionEnum::RIGHT_WHEEL_RADIUS,
			rightWheelRadius);
	}

	if( pszWheelBaseLength )
	{
		double wheelbaseLength = 0.0f;
		sscanf( pszWheelBaseLength, "%lf", &wheelbaseLength );

		messageProcessor_.setDimension(SetPhysicalDimension::DimensionEnum::WHEEL_BASE_LENGTH,
			wheelbaseLength);
	}

	brainstemFaultTimer_ = nodeHandle_.createTimer(ros::Duration(1.0f / REFRESH_RATE_HZ),
        boost::bind(&BrainStemMessageProcessor::checkForBrainstemFaultTimer,
        	&messageProcessor_, _1));
}

BrainStem::~BrainStem( )
{

}

void BrainStem::execute()
{
	io_->spinOnce();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void BrainStem::initialize()
{

}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Callbacks
////////////////////////////////////////////////////////////////////////////////////////////////////

void BrainStem::connectionChanged( bool bIsConnected )
{
	messageProcessor_.setConnected(bIsConnected);
	if( useEmulator_ )
	{
		if( !bIsConnected )
		{
			brainstemEmulator_.reset( new BrainStemEmulator( ) );

			uint16_t uniqueId[8] = { 0x3100, 0x5335, 0x4B50, 0x5948, 0x3331, 0x3130, 0x3032, 0x3331 };

			// Emulate the hardware info to kick start the rest of the system (mfp_bridge, mfp_config_client, mfp_gui)
			// TODO: Emulate the rest of the brainstem: https://github.com/6RiverSystems/ros/issues/27
			std::string version("Brainstem Emulator");

			vector<char> buffer;
			HardwareMessage msg(buffer);
			msg.write<uint8_t>(static_cast<uint8_t>(BRAIN_STEM_MSG::HARDWARE_INFO));
			msg.writeArray<uint16_t>(uniqueId, 8);
			msg.write<uint8_t>(static_cast<uint8_t>(1));
			msg.write<uint8_t>(static_cast<uint8_t>(2));
			msg.write(version);

			messageProcessor_.processHardwareMessage(buffer);
		}
		else
		{
			brainstemEmulator_.reset( );
		}
	}
}

void BrainStem::cfgCallback(srsdrv_brainstem::RobotSetupConfig &config,
	uint32_t level)
{
	IO_TYPE ioType = static_cast<IO_TYPE>(config.io_type);

	if (ioType == IO_TYPE::HID)
	{
		setupHidIo(config.io_vid, config.io_pid);
	}
	else
	{
		setupSerialIo(config.io_serial_device.c_str());
	}

	messageProcessor_.setUseBrainstemOdom(config.use_brainstem_odom);

	messageProcessor_.setDimension(SetPhysicalDimension::DimensionEnum::WHEEL_BASE_LENGTH,
		static_cast<float>(config.robot_wheelbase_length));

	messageProcessor_.setDimension(SetPhysicalDimension::DimensionEnum::LEFT_WHEEL_RADIUS,
		static_cast<float>(config.robot_leftwheel_radius));

	messageProcessor_.setDimension(SetPhysicalDimension::DimensionEnum::RIGHT_WHEEL_RADIUS,
		static_cast<float>(config.robot_rightwheel_radius));
}

void BrainStem::setupHidIo(uint32_t vid, uint32_t pid)
{
	ROS_DEBUG("Brainstem driver: Using hid io: vid=0x%x, pid=0x%x", vid, pid);

	io_.reset(new HidIO("brainstem", vid,  pid));

	messageProcessor_.setIO(io_);

	io_->open( std::bind( &BrainStem::connectionChanged, this, std::placeholders::_1 ),
		std::bind( &BrainStemMessageProcessor::processHardwareMessage, &messageProcessor_, std::placeholders::_1) );
}

void BrainStem::setupSerialIo(const char* serialPort)
{
	ROS_DEBUG("Brainstem driver: Using serial port io: %s", serialPort);

	io_.reset(new SerialIO("brainstem", serialPort));

	std::shared_ptr<SerialIO> serialIO = std::dynamic_pointer_cast<SerialIO>( io_ );

	serialIO->enableCRC(true);
	serialIO->setTerminatingCharacter('\n');
	serialIO->setEscapeCharacter('\\');

    auto processMessage = [&]( std::vector<char> buffer )
    {
        ExecuteInRosThread( std::bind( &BrainStemMessageProcessor::processHardwareMessage,
            &messageProcessor_, buffer));
	};

	auto connectionChanged = [&]( bool bIsConnected )
	{
		ExecuteInRosThread(std::bind(&BrainStem::connectionChanged, this,
			bIsConnected));
	};

	serialIO->open(connectionChanged, processMessage);

	messageProcessor_.setIO(io_);
}

}// namespace srs
