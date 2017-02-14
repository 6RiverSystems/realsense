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
        upTime(0),
        frontEStop(false),
        backEStop(false),
        wirelessEStop(false),
        bumpSensor(false),
        freeSpin(false),
        hardStop(false),

        safetyProcessorFailure(false),
        brainstemFailure(false),
        brainstemTimeout(false),
        brainTimeout(false),
        rightMotorController(false),
        leftMotorController(false),
        rpmMessageTimeout(false),
        velocityViolation(false)
    {}

    virtual ~RobotState()
    {}

    bool any()
    {
        return anyMotionStatus() || anyFailureStatus();
    }

    bool anyEStop()
    {
        return frontEStop || backEStop || wirelessEStop;
    }

    bool anyMotionStatus()
    {
        return frontEStop || backEStop || wirelessEStop ||
            bumpSensor || freeSpin || hardStop;
    }

    bool anyFailureStatus()
    {
        return safetyProcessorFailure || brainstemFailure || brainTimeout || brainstemTimeout ||
            rightMotorController || leftMotorController ||
            rpmMessageTimeout || velocityViolation;
    }

    friend bool operator==(const RobotState& lhs, const RobotState& rhs)
    {
        return lhs.upTime == rhs.upTime &&
            lhs.frontEStop == rhs.frontEStop &&
            lhs.backEStop == rhs.backEStop &&
            lhs.wirelessEStop == rhs.wirelessEStop &&
            lhs.bumpSensor == rhs.bumpSensor &&
            lhs.freeSpin == rhs.freeSpin &&
            lhs.hardStop == rhs.hardStop &&
            lhs.safetyProcessorFailure == rhs.safetyProcessorFailure &&
            lhs.brainstemFailure == rhs.brainstemFailure &&
            lhs.brainstemTimeout == rhs.brainstemTimeout &&
            lhs.brainTimeout == rhs.brainTimeout &&
            lhs.rightMotorController == rhs.rightMotorController &&
            lhs.leftMotorController == rhs.leftMotorController &&
            lhs.rpmMessageTimeout == rhs.rpmMessageTimeout &&
            lhs.velocityViolation == rhs.velocityViolation;
    }

    friend ostream& operator<<(ostream& stream, const RobotState& robotState)
    {
        stream << "RobotState {" <<
            "uptime: " << robotState.upTime <<
            ", MotionStatus {"
            "frontEStop: " << (robotState.frontEStop ? "true" : "false")  <<
            ", backEStop: " << (robotState.backEStop ? "true" : "false")  <<
            ", wirelessEStop: " << (robotState.wirelessEStop ? "true" : "false")  <<
            ", bumpSensor: " << (robotState.bumpSensor ? "true" : "false")  <<
            ", freeSpin: " << (robotState.freeSpin ? "true" : "false")  <<
            ", hardStop: " << (robotState.hardStop ? "true" : "false")  <<

            "}, FailureStatus {"
            "safetyProcessorFailure: " << (robotState.safetyProcessorFailure ? "true" : "false")  <<
            ", brainstemFailure: " << (robotState.brainstemFailure ? "true" : "false")  <<
            ", brainstemTimeout: " << (robotState.brainstemTimeout ? "true" : "false")  <<
            ", brainTimeout: " << (robotState.brainTimeout ? "true" : "false")  <<
            ", rightMotorController: " << (robotState.rightMotorController ? "true" : "false")  <<
            ", leftMotorController: " << (robotState.leftMotorController ? "true" : "false") <<
            ", rpmMessageTimeout: " << (robotState.rpmMessageTimeout ? "true" : "false") <<
            ", velocityViolation: " << (robotState.velocityViolation ? "true" : "false") <<
            "}}";

        return stream;
    }

    unsigned int upTime;

    // Motion Status
    bool frontEStop;
    bool backEStop;
    bool wirelessEStop;
    bool bumpSensor;
    bool freeSpin;
    bool hardStop;

    // Failure Status
    bool safetyProcessorFailure;
    bool brainstemFailure;
    bool brainstemTimeout;
    bool brainTimeout;
    bool rightMotorController;
    bool leftMotorController;
    bool rpmMessageTimeout;
    bool velocityViolation;
};

} // namespace srs
