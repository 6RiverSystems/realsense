/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <algorithm>
using namespace std;

#include <srslib_framework/robotics/Velocity.hpp>
#include <srslib_framework/ros/tap/subscriber/SubscriberJoypadState.hpp>
#include <srslib_framework/ros/topics/ChuckTopics.hpp>

namespace srs {

class TapJoypad :
    public SubscriberJoypadState
{
public:
    TapJoypad() :
        SubscriberJoypadState(ChuckTopics::sensor::JOYPAD_STATE, 1000)
    {}

    ~TapJoypad()
    {}

    bool getButtonAction()
    {
        return pop().buttonAction;
    }

    bool getButtonEmergency()
    {
        return pop().buttonEmergency;
    }

    bool getButtonX()
    {
        return pop().buttonX;
    }

    bool getButtonY()
    {
        return pop().buttonY;
    }

    bool getLatchState()
    {
        return pop().latched;
    }

    Velocity<> getVelocity()
    {
        return pop().velocity;
    }
};

} // namespace srs
