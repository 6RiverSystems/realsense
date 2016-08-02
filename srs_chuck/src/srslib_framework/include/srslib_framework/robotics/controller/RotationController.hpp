/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef ROTATIONCONTROLLER_HPP_
#define ROTATIONCONTROLLER_HPP_

#include <srslib_framework/math/AngleMath.hpp>
#include <srslib_framework/math/BasicMath.hpp>

#include <srslib_framework/robotics/controller/BaseController.hpp>

namespace srs {

class RotationController: public BaseController
{
public:
    RotationController() :
        BaseController("ROTATION CONTROLLER"),
        integral_(0.0),
        previousError_(0.0)
    {}

    ~RotationController()
    {}

    void reset()
    {
        BaseController::reset();

        integral_ = 0.0;
        previousError_ = 0.0;
    }

    void setRobotProfile(RobotProfile robot)
    {
        BaseController::setRobotProfile(robot, robot.rotationKv, robot.rotationKw);
    }

protected:
    void stepController(double dT, Pose<> currentPose, Odometry<> currentOdometry)
    {
        // The rotation controller ignores the canceled signal
        // so that the rotation can be completed

        // Calculate the difference between what has been requested and
        // the current pose
        double error = AngleMath::normalizeAngleRad<double>(getGoal().theta - currentPose.theta);

        if (!isRobotMoving() &&
            AngleMath::equalRad<double>(getGoal().theta, currentPose.theta,
                robot_.rotationGoalReachedAngle))
        {
            setGoalReached(true);
            executeCommand(Velocity<>::ZERO);

            return;
        }

        // Calculate the linear portion of the command
        double linear = 0;

        // Calculate the angular portion of the command
        double angular = robot_.rotationKp * error + robot_.rotationKi * integral_ +
            robot_.rotationKd * (error - previousError_);

        integral_ += error;
        previousError_ = error;

        // Make sure that the calculated angular velocity is not bigger
        // than the specified travel angular velocity
        angular = BasicMath::saturate<double>(angular,
            robot_.rotationRotationVelocity, -robot_.rotationRotationVelocity);

        // Send the command for execution
        executeCommand(linear, angular);
    }

private:
    double integral_;
    double previousError_;
};

} // namespace srs

#endif // ROTATIONCONTROLLER_HPP_
