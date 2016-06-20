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
        Kd_(0.0),
        Ki_(0.0),
        Kp_(1.0),
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

    void setPIDGains(double Kp, double Ki, double Kd)
    {
        Kp_ = Kp;
        Ki_ = Ki;
        Kd_ = Kd;
    }

protected:
    void stepController(double dT, Pose<> currentPose, Odometry<> currentOdometry)
    {
        // The rotation controller ignores the canceled signal
        // so that the rotation can be completed

        // Calculate the difference between what has been requested and
        // the current pose
        double error = AngleMath::normalizeAngleRad<double>(goal_.theta - currentPose.theta);

        if (AngleMath::equalRad<double>(goal_.theta, currentPose.theta, robot_.goalReachedAngle))
        {
            goalReached_ = true;
        }

        // Calculate the linear portion of the command
        double linear = 0;

        // Calculate the angular portion of the command
        double angular = Kw_ * (Kp_ * error + Ki_ * integral_ + Kd_ * (error - previousError_));

        angular = BasicMath::saturate<double>(angular,
            robot_.travelRotationVelocity, -robot_.travelRotationVelocity);

        angular = BasicMath::threshold<double>(angular,
            robot_.minPhysicalAngularVelocity, 0.0);

        integral_ += error;
        previousError_ = error;

        // Send the command for execution
        executeCommand(Velocity<>(linear, angular));
    }

private:
    double Kd_;
    double Ki_;
    double Kp_;

    double integral_;
    double previousError_;
};

} // namespace srs

#endif // ROTATIONCONTROLLER_HPP_
