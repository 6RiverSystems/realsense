/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#include <BrainStemMessageProcessorInterface.hpp>

#include <srslib_framework/ros/topics/ChuckTopics.hpp>

#include <sw_message/SetVelocityHandler.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
SetVelocityHandler::SetVelocityHandler(BrainStemMessageProcessorInterface* owner) :
    SoftwareMessageHandler(owner),
	SubscriberRosTwist(ChuckTopics::driver::ODOMETRY_CMD_VELOCITY)
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void SetVelocityHandler::attach()
{
	// TODO: Add to base subscriber class (delay ros intantiation for unit tests)
}

void SetVelocityHandler::receiveData(const geometry_msgs::Twist::ConstPtr data)
{
	SetVelocityData msg = {
		static_cast<uint8_t>( BRAIN_STEM_CMD::SET_VELOCITY ),
		static_cast<float>( data->linear.x ),
		static_cast<float>( data->angular.z )
	};

	ROS_INFO_NAMED("velocity", "Brain => Brainstem: Set velocity: linear=%f, angular=%f",
		data->linear.x, data->angular.z);

	getOwner()->sendCommand( reinterpret_cast<char*>( &msg ), sizeof(msg) );
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods


} // namespace srs
