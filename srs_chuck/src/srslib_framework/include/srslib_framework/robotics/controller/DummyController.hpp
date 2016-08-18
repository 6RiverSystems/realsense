/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/robotics/controller/BaseController.hpp>

namespace srs {

class DummyController: public BaseController
{
public:
    DummyController() :
        BaseController("DUMMY CONTROLLER")
    {}

    ~DummyController()
    {}

    void setRobotProfile(RobotProfile robot)
    {
        BaseController::setRobotProfile(robot, 1.0, 1.0);
    }

protected:
    void stepController(double dT, Pose<> currentPose, Odometry<> currentOdometry)
    {
        // The controller never completes its goal. In order to complete its
        // goal, it must be canceled
        setGoalReached(false);
    }

private:
};

} // namespace srs
