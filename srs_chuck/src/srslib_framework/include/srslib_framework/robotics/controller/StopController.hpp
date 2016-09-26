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
        BaseController("STOP CONTROLLER"),
        currentLinearVelocity_()
    {}

    ~StopController()
    {}

    void reset()
    {
        BaseController::reset();

        currentLinearVelocity_ = 0.0;
    }

    void setRobotProfile(RobotProfile robot)
    {
        BaseController::setRobotProfile(robot, 1.0, 1.0);
    }

protected:
    void stepController(double dT, Pose<> currentPose, Odometry<> currentOdometry)
    {
        // If this is the first run of the controller, store the current
        // velocity so that it can be slowly forced to 0
        if (isFirstRun())
        {
            currentLinearVelocity_ = currentOdometry.velocity.linear;

            ROS_DEBUG_STREAM_NAMED("controller_stop", "Set linear velocity: " <<
                currentLinearVelocity_);
        }

        // If the robot is not moving anymore, the goal was reached
        if (!isRobotMoving())
        {
            setGoalReached(true);
            executeCommand(Velocity<>::ZERO);

            return;
        }

        double deltaVelocity = robot_.stopNormalDeceleration * dT;
        currentLinearVelocity_ -=  BasicMath::sgn<double>(currentLinearVelocity_) * deltaVelocity;

        // If the new linear velocity is below the specified
        // stop minimum linear velocity, force it directly to 0
        currentLinearVelocity_ = BasicMath::threshold<double>(currentLinearVelocity_,
            robot_.stopMinLinearVelocity, 0.0);

        // Send the command for execution
        executeCommand(currentLinearVelocity_, 0.0);
    }

private:
    double currentLinearVelocity_;
};

} // namespace srs

#endif // STOPCONTROLLER_HPP_
