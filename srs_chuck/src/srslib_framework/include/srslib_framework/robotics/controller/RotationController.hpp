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
        BaseController(),
        Kd_(0.0),
        Ki_(0.0),
        Kp_(0.0),
        integral_(0.0),
        previousError_(0.0)
    {}

    ~RotationController()
    {}

    void setPIDGains(double Kp, double Ki, double Kd)
    {
        Kp_ = Kp;
        Ki_ = Ki;
        Kd_ = Kd;
    }

    Velocity<> stepController(Pose<> currentPose, Odometry<> currentOdometry)
    {
        double error = AngleMath::normalizeAngleRad(desiredPose.theta - currentPose.theta);

        // Calculate the linear portion of the command
        double linear = Kv_ * command.linear;
        linear = BasicMath::saturate<double>(linear, maxLinear_, -maxLinear_);

        // Calculate the angular portion of the command
        double angular = Kw_ * (Kp_ * error + Ki_ * integral_ + Kd_ * (error - previousError_));
        angular = BasicMath::saturate<double>(angular, maxAngular_, -maxAngular_);

        integral_ += error;
        previousError_ = error;

        return Velocity<>(linear, angular);
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
