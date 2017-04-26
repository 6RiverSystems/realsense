/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
using namespace std;

#include <ros/ros.h>
#include <srslib_framework/MsgUpdateUI.h>

#include <srslib_framework/robotics/device/Sound.hpp>
#include <srslib_framework/ros/tap/subscriber/SubscriberSingleData.hpp>
#include <srslib_framework/ros/message/SoundMessageFactory.hpp>

namespace srs {

class SubscriberUpdateUI :
    public SubscriberSingleData<srslib_framework::MsgUpdateUI, srslib_framework::MsgUpdateUI>
{
public:
    SubscriberUpdateUI(string topic,
        unsigned int queueLength = 10,
        string nameSpace = "~") :
            SubscriberSingleData(topic, queueLength, nameSpace)
    {
        reset();
    }

    ~SubscriberUpdateUI()
    {}

    void receiveData(const srslib_framework::MsgUpdateUI::ConstPtr message)
    {
        set(*message);
    }
};

} // namespace srs
