/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/MsgOperationalState.h>

#include <srslib_framework/robotics/device/RobotState.hpp>

namespace srs {

struct RobotStateMessageFactory
{
    /**
     * @brief Convert a RobotState type into a MsgOperationalState.
     *
     * @param robotState RobotState to convert
     *
     * @return newly generated message
     */
    static srslib_framework::MsgOperationalState robotState2Msg(const RobotState& robotState)
    {
        srslib_framework::MsgOperationalState msgOperationalState;

        msgOperationalState.upTime = robotState.upTime;
        msgOperationalState.frontEStop = robotState.frontEStop;
        msgOperationalState.backEStop = robotState.backEStop;
        msgOperationalState.wirelessEStop = robotState.wirelessEStop;
        msgOperationalState.bumpSensor = robotState.bumpSensor;
        msgOperationalState.pause = robotState.freeSpin;
        msgOperationalState.hardStop = robotState.hardStop;

        msgOperationalState.safetyProcessorFailure = robotState.safetyProcessorFailure;
        msgOperationalState.brainstemFailure = robotState.brainstemFailure;
        msgOperationalState.brainTimeoutFailure = robotState.brainTimeout;
        msgOperationalState.rightMotorFailure = robotState.rightMotorController;
        msgOperationalState.leftMotorFailure = robotState.leftMotorController;
        msgOperationalState.rpmMessageTimeout = robotState.rpmMessageTimeout;
        msgOperationalState.velocityViolation = robotState.velocityViolation;

        return msgOperationalState;
    }

    /**
     * @brief Convert a MsgOperationalState type into a RobotState.
     *
     * @param message Message to convert
     *
     * @return newly generated RobotState
     */
    static RobotState msg2RobotState(const srslib_framework::MsgOperationalState& message)
    {
        RobotState robotState;

        robotState.upTime = message.upTime;

        robotState.frontEStop = message.frontEStop;
        robotState.backEStop = message.backEStop;
        robotState.wirelessEStop = message.wirelessEStop;
        robotState.bumpSensor = message.bumpSensor;
        robotState.freeSpin = message.pause;
        robotState.hardStop = message.hardStop;

        robotState.safetyProcessorFailure = message.safetyProcessorFailure;
        robotState.brainstemFailure = message.brainstemFailure;
        robotState.brainTimeout = message.brainTimeoutFailure;
        robotState.rightMotorController = message.rightMotorFailure;
        robotState.leftMotorController = message.leftMotorFailure;
        robotState.rpmMessageTimeout = message.rpmMessageTimeout;
        robotState.velocityViolation = message.velocityViolation;

        return robotState;
    }

    /**
     * @brief Convert a MsgOperationalStateConstPtr type into a RobotState.
     *
     * @param message Message to convert
     *
     * @return newly generated RobotState
     */
    static RobotState msg2RobotState(srslib_framework::MsgOperationalState::ConstPtr message)
    {
        return msg2RobotState(*message);
    }
};

} // namespace srs
