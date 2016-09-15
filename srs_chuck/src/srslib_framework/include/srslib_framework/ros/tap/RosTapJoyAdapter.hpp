/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <algorithm>
using namespace std;

#include <srslib_framework/robotics/Velocity.hpp>
#include <srslib_framework/ros/subscriber/SubscriberSrsJoypadState.hpp>

namespace srs {

class RosTapJoyAdapter :
    public SubscriberSrsJoypadState
{
public:
    RosTapJoyAdapter() :
        SubscriberSrsJoypadState("/internal/sensors/joystick/state", 1000)
    {}

    ~RosTapJoyAdapter()
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
