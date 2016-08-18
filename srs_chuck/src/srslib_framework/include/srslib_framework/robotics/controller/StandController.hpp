/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef STANDCONTROLLER_HPP_
#define STANDCONTROLLER_HPP_

#include <srslib_framework/robotics/controller/BaseController.hpp>

namespace srs {

class StandController: public BaseController
{
public:
    StandController() :
        BaseController("STAND CONTROLLER")
    {}

    ~StandController()
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

        // Send the command for execution
        executeCommand(Velocity<>::ZERO);
    }

private:
};

} // namespace srs

#endif // STANDCONTROLLER_HPP_
