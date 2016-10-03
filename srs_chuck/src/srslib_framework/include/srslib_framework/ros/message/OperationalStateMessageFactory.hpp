/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef OPERATIONALSTATEMESSAGEFACTORY_HPP_
#define OPERATIONALSTATEMESSAGEFACTORY_HPP_

#include <srslib_framework/MsgOperationalState.h>

#include <srslib_framework/robotics/RobotState.hpp>

namespace srs {

struct OperationalStateMessageFactory
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
        srslib_framework::MsgOperationalState msgOperationState;

        msgOperationState.frontEStop = robotState.frontEStop;
        msgOperationState.backEStop = robotState.backEStop;
        msgOperationState.wirelessEStop = robotState.wirelessEStop;
        msgOperationState.bumpSensor = robotState.bumpSensor;
        msgOperationState.pause = robotState.pause;
        msgOperationState.hardStop = robotState.hardStop;
        msgOperationState.safetyProcessorFailure = robotState.safetyProcessorFailure;
        msgOperationState.brainstemFailure = robotState.brainstemFailure;
        msgOperationState.brainTimeoutFailure = robotState.brainTimeoutFailure;
        msgOperationState.rightMotorFailure = robotState.rightMotorFailure;
        msgOperationState.leftMotorFailure = robotState.leftMotorFailure;

        return msgOperationState;
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

        robotState.frontEStop = message.frontEStop;
        robotState.backEStop = message.backEStop;
        robotState.wirelessEStop = message.wirelessEStop;
        robotState.bumpSensor = message.bumpSensor;
        robotState.pause = message.pause;
        robotState.hardStop = message.hardStop;
        robotState.safetyProcessorFailure = message.safetyProcessorFailure;
        robotState.brainstemFailure = message.brainstemFailure;
        robotState.brainTimeoutFailure = message.brainTimeoutFailure;
        robotState.rightMotorFailure = message.rightMotorFailure;
        robotState.leftMotorFailure = message.leftMotorFailure;

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
        return OperationalStateMessageFactory::msg2RobotState(*message);
    }
};

} // namespace srs

#endif // OPERATIONALSTATEMESSAGEFACTORY_HPP_
