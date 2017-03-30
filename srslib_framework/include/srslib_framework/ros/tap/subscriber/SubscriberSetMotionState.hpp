/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
using namespace std;

#include <ros/ros.h>
#include <srslib_framework/MsgSetOperationalState.h>

#include <srslib_framework/ros/tap/subscriber/SubscriberSingleData.hpp>

namespace srs {

class SubscriberSetMotionState :
    public SubscriberSingleData<srslib_framework::MsgSetOperationalState, srslib_framework::MsgSetOperationalState>
{
public:
    SubscriberSetMotionState(string topic,
        unsigned int queueLength = 10,
        string nameSpace = "~") :
            SubscriberSingleData(topic, queueLength, nameSpace)
    {
        reset();
    }

    ~SubscriberSetMotionState()
    {}

    void receiveData(const srslib_framework::MsgSetOperationalState::ConstPtr message)
    {
        set(*message);
    }
};

} // namespace srs
