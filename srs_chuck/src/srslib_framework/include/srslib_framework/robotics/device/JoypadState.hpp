/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/robotics/Velocity.hpp>

namespace srs {

struct JoypadState
{
    JoypadState() :
        arrivalTime(0.0),
        connected(false),
        buttonEmergency(false),
        buttonAction(false),
        buttonX(false),
        buttonY(false),
        velocity(Velocity<>::INVALID),
        latched(false)
    {}

    virtual ~JoypadState()
    {}

    bool anyButton()
    {
        return buttonEmergency || buttonAction;
    }

    double arrivalTime;

    bool buttonAction;
    bool buttonEmergency;
    bool buttonX;
    bool buttonY;

    bool connected;

    Velocity<> velocity;

    bool latched;
};

} // namespace srs
