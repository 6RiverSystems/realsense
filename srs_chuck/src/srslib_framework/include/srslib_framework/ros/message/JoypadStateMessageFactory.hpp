/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/JoypadState.h>

#include <srslib_framework/robotics/device/JoypadState.hpp>
#include <srslib_framework/ros/message/VelocityMessageFactory.hpp>

namespace srs {

struct JoypadStateMessageFactory
{
    /**
     * @brief Convert a JoypadState type into a JoypadState message.
     *
     * @param joypadState JoypadState to convert
     *
     * @return newly generated message
     */
    static srslib_framework::JoypadState joypadState2Msg(const JoypadState& joypadState)
    {
        srslib_framework::JoypadState msgJoypadState;

        msgJoypadState.arrivalTime = joypadState.arrivalTime;
        msgJoypadState.latched = joypadState.latched;
        msgJoypadState.connected = joypadState.connected;
        msgJoypadState.velocity = VelocityMessageFactory::velocity2Msg(joypadState.velocity);
        msgJoypadState.buttonAction = joypadState.buttonAction;
        msgJoypadState.buttonEmergency = joypadState.buttonEmergency;
        msgJoypadState.buttonX = joypadState.buttonX;
        msgJoypadState.buttonY = joypadState.buttonY;

        return msgJoypadState;
    }

    /**
     * @brief Convert a JoypadState message type into a JoypadState.
     *
     * @param message Message to convert
     *
     * @return newly generated JoypadState
     */
    static JoypadState msg2JoypadState(const srslib_framework::JoypadState& message)
    {
        JoypadState joypadState;

        joypadState.arrivalTime = message.arrivalTime;
        joypadState.latched = message.latched;
        joypadState.connected = message.connected;
        joypadState.velocity = VelocityMessageFactory::msg2Velocity(message.velocity);
        joypadState.buttonAction = message.buttonAction;
        joypadState.buttonEmergency = message.buttonEmergency;
        joypadState.buttonX = message.buttonX;
        joypadState.buttonY = message.buttonY;

        return joypadState;
    }

    /**
     * @brief Convert a JoypadState message type into a JoypadState.
     *
     * @param message Message to convert
     *
     * @return newly generated RobotState
     */
    static JoypadState msg2JoypadState(srslib_framework::JoypadState::ConstPtr message)
    {
        return JoypadStateMessageFactory::msg2JoypadState(*message);
    }
};

} // namespace srs
