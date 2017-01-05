/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <ros/ros.h>

#include <BrainStemMessageProcessorInterface.hpp>

#include <hw_message/HardwareMessage.hpp>

namespace srs {

class HardwareMessageHandler
{
public:
    HardwareMessageHandler(BRAIN_STEM_MSG messageKey) :
		messageKey_(messageKey)
    {
    }

    virtual ~HardwareMessageHandler() {}

    BRAIN_STEM_MSG getKey() const
    {
        return messageKey_;
    }

    bool isKeyMatching(BRAIN_STEM_MSG key) const
    {
        return key == messageKey_;
    }

    void receiveData(ros::Time currentTime, vector<char>& msg)
    {
    	HardwareMessage hardwareMessage(msg);

    	receiveMessage(currentTime, hardwareMessage);
    }

    virtual void receiveMessage(ros::Time currentTime, HardwareMessage& msg) = 0;

private:

    BRAIN_STEM_MSG		messageKey_;
};

typedef std::shared_ptr<HardwareMessageHandler> HardwareMessageHandlerPtr;

} // namespace srs
