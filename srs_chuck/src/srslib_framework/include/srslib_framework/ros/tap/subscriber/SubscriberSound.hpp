/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
using namespace std;

#include <ros/ros.h>
#include <srslib_framework/Sound.h>

#include <srslib_framework/robotics/device/Sound.hpp>
#include <srslib_framework/ros/tap/subscriber/SubscriberSingleData.hpp>
#include <srslib_framework/ros/message/SoundMessageFactory.hpp>

namespace srs {

class SubscriberSound :
    public SubscriberSingleData<srslib_framework::Sound, Sound>
{
public:
    SubscriberSound(string topic,
        unsigned int queueLength = 10,
        string nameSpace = "~") :
            SubscriberSingleData(topic, queueLength, nameSpace)
    {
        reset();
    }

    ~SubscriberSound()
    {}

    void receiveData(const srslib_framework::Sound::ConstPtr message)
    {
        set(SoundMessageFactory::msg2Sound(message));
    }
};

} // namespace srs
