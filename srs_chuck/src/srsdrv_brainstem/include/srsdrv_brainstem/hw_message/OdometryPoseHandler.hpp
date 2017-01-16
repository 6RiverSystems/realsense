/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <tf/transform_broadcaster.h>

#include <BrainStemMessages.hpp>
#include <hw_message/HardwareMessageHandler.hpp>

#include <srslib_framework/ros/channel/ChannelBrainstemOdometryPose.hpp>

namespace srs {

class OdometryPoseHandler : public HardwareMessageHandler
{
public:
    OdometryPoseHandler(ChannelBrainstemOdometryPose::Interface& publisher, bool useBrainstemOdom);

    virtual ~OdometryPoseHandler() {}

    void receiveMessage(ros::Time currentTime, HardwareMessage& msg);

private:
    HW_MESSAGE_BEGIN(OdometryPoseData)
		uint8_t cmd;
		uint32_t timestamp;
		float linearVelocity;
		float angularVelocity;
		float x;
		float y;
		float theta;
	HW_MESSAGE_END

	tf::TransformBroadcaster broadcaster_;

	PublisherOdometryPose::Interface& publisher_;

	bool useBrainstemOdom_;
};

} // namespace srs
