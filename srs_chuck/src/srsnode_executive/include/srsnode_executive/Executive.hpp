/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/ros/tap/TapOdometryCmd_Velocity.hpp>
#include <srslib_framework/ros/tap/TapRobotPose.hpp>
#include <srslib_framework/ros/tap/TapMapStack.hpp>
#include <srslib_framework/ros/unit/RosUnit.hpp>

#include <srsnode_executive/ExecutiveContext.hpp>
#include <srsnode_executive/task/TaskDetectLabeledAreas.hpp>
#include <srsnode_executive/task/TaskPlayWarningSound.hpp>

namespace srs {

class Executive : public RosUnit<Executive>
{
public:
    Executive(string name, int argc, char** argv);
    ~Executive()
    {}

    void setPauseState(bool newState)
    {
        context_.isRobotPaused = newState;
    }

protected:
    void execute();

    void initialize();

private:
    constexpr static double REFRESH_RATE_HZ = 5; // [Hz]

    void updateContext();

    ExecutiveContext context_;

    TapOdometryCmd_Velocity tapCommandedVelocity_;
    TapMapStack tapMapStack_;
    TapRobotPose tapRobotPose_;
    TaskDetectLabeledAreas taskDetectLabeledAreas_;
    TaskPlayWarningSound taskPlayWarningSound_;
};

} // namespace srs
