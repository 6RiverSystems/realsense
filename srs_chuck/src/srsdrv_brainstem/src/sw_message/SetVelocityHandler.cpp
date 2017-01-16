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
	SoftwareMessage(owner),
	SubscriberRosTwist(ChuckTopics::driver::ODOMETRY_CMD_VELOCITY)
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void SetVelocityHandler::receiveData(const geometry_msgs::Twist::ConstPtr twist)
{
	twist_.reset( new geometry_msgs::Twist(*twist));

	SetVelocityData msg = {
		static_cast<uint8_t>( BRAIN_STEM_CMD::SET_VELOCITY ),
		static_cast<float>( twist->linear.x ),
		static_cast<float>( twist->angular.z )
	};

//	ROS_DEBUG_NAMED("velocity", "Brain => Brainstem: Set velocity: linear=%f, angular=%f",
//		data->linear.x, data->angular.z);

//	sendCommand(reinterpret_cast<char*>( &msg ), sizeof(msg));
}

void SetVelocityHandler::syncState()
{
	if (twist_)
	{
		receiveData(twist_);
	}
}
////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods


} // namespace srs
