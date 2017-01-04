/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <BrainStemMessages.hpp>
#include <sw_message/SoftwareMessage.hpp>

#include <srslib_framework/ros/tap/subscriber/Subscriber.hpp>
#include <srslib_framework/platform/observer/Observer.hpp>
#include <srslib_framework/ros/tap/subscriber/SubscriberRosTwist.hpp>

namespace srs {

class BrainStemMessageProcessorInterface;

class SetVelocityHandler :
	public SoftwareMessage,
    public SubscriberRosTwist
{
public:
    SetVelocityHandler(BrainStemMessageProcessorInterface* owner);

    virtual ~SetVelocityHandler() {}

    virtual void receiveData(const geometry_msgs::Twist::ConstPtr data);

    void syncState();

private:

    HW_MESSAGE_BEGIN(SetVelocityData)
		uint8_t cmd;
		float linearVelocity;
		float angularVelocity;
	HW_MESSAGE_END

	geometry_msgs::Twist::ConstPtr twist_;
};

} // namespace srs
