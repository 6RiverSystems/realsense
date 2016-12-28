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
#include <srslib_framework/ros/topics/ChuckTopics.hpp>

namespace srs
{

BrainStem::BrainStem(string name, int argc, char** argv) :
	RosUnit(name, argc, argv, REFRESH_RATE_HZ),
	io_( new HidIO( "brainstem", 0x1930, 0x6001 ) ),
	messageProcessor_( io_ ),
	brainstemFaultTimer_(),
	nodeHandle_("~")
{
	// Register dynamic configuration callback
	configServer_.setCallback(boost::bind(&BrainStem::cfgCallback, this, _1, _2));

	connectionChanged( false );

	std::shared_ptr<HidIO> pSerialIO = std::dynamic_pointer_cast<HidIO>( io_ );

    auto processMessage = std::bind( &BrainStemMessageProcessor::processHardwareMessage,
    	&messageProcessor_, std::placeholders::_1);

	auto connectionChanged = std::bind( &BrainStem::connectionChanged, this,
		std::placeholders::_1 );

	pSerialIO->open(connectionChanged, processMessage);

    // Check for hardware faults
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

//	if( !bIsConnected )
//	{
//		brainstemEmulator_.reset( new BrainStemEmulator( ) );
//	}
//	else
//	{
//		brainstemEmulator_.reset( );
//	}
}

void BrainStem::cfgCallback(srsdrv_brainstem::RobotSetupConfig &config,
	uint32_t level)
{
	messageProcessor_.setDimension(BrainStemMessageProcessor::DIMENSION::WHEEL_BASE_LENGTH,
		static_cast<float>(config.robot_wheelbase_length));

	messageProcessor_.setDimension(BrainStemMessageProcessor::DIMENSION::LEFT_WHEEL_RADIUS,
		static_cast<float>(config.robot_leftwheel_radius));

	messageProcessor_.setDimension(BrainStemMessageProcessor::DIMENSION::RIGHT_WHEEL_RADIUS,
		static_cast<float>(config.robot_rightwheel_radius));
}

}// namespace srs
