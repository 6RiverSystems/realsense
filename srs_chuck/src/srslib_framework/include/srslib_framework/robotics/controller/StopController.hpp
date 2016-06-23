/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef STOPCONTROLLER_HPP_
#define STOPCONTROLLER_HPP_

#include <srslib_framework/math/AngleMath.hpp>
#include <srslib_framework/math/BasicMath.hpp>

#include <srslib_framework/robotics/controller/BaseController.hpp>

namespace srs {

class StopController: public BaseController
{
public:
    StopController() :
        BaseController("STOP CONTROLLER")
    {}

    ~StopController()
    {}

    void setRobotProfile(RobotProfile robot)
    {
        BaseController::setRobotProfile(robot, 1.0, 1.0);
    }

protected:
    void stepController(double dT, Pose<> currentPose, Odometry<> currentOdometry)
    {
        // If the robot is not moving anymore, the goal was reached
        if (!isRobotMoving_)
        {
            goalReached_ = true;
            executeCommand(ZERO_VELOCITY);

            return;
        }

        double deltaVelocity = robot_.travelLinearAcceleration * dT;

        double linear = currentOdometry.velocity.linear;
        linear -=  BasicMath::sgn<double>(linear) * deltaVelocity;
        linear = BasicMath::threshold<double>(linear, robot_.minLinearVelocity, 0.0);

        // Send the command for execution
        executeCommand(linear, 0.0);
    }
};

} // namespace srs

#endif // STOPCONTROLLER_HPP_
