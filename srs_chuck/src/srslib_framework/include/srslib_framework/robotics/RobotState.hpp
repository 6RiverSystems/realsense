/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

namespace srs {

struct RobotState
{
    RobotState() :
        frontEStop(false),
        backEStop(false),
        wirelessEStop(false),
        bumpSensor(false),
        pause(false),
        hardStop(false),
        safetyProcessorFailure(false),
        brainstemFailure(false),
        brainTimeoutFailure(false),
        rightMotorFailure(false),
        leftMotorFailure(false)
    {}

    virtual ~RobotState()
    {}

    bool any()
    {
        return anyEStop() ||
            bumpSensor || pause || hardStop ||
            anyFailureSignal();
    }

    bool anyEStop()
    {
        return frontEStop || backEStop || wirelessEStop;
    }

    bool anyFailureSignal()
    {
        return safetyProcessorFailure || brainstemFailure || brainTimeoutFailure ||
            rightMotorFailure || leftMotorFailure;
    }

    bool frontEStop;
    bool backEStop;
    bool wirelessEStop;
    bool bumpSensor;
    bool pause;
    bool hardStop;
    bool safetyProcessorFailure;
    bool brainstemFailure;
    bool brainTimeoutFailure;
    bool rightMotorFailure;
    bool leftMotorFailure;
};

} // namespace srs
