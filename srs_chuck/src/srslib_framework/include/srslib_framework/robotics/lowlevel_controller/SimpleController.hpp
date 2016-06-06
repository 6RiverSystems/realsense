/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef SIMPLECONTROLLER_HPP_
#define SIMPLECONTROLLER_HPP_

#include <srslib_framework/math/Math.hpp>

#include <srslib_framework/robotics/lowlevel_controller/BaseController.hpp>

namespace srs {

class SimpleController: public BaseController
{
public:
    SimpleController(double Kv, double Kw) :
        BaseController(Kv, Kw),
        lookAheadDistance_(0.5)
    {}

    ~SimpleController()
    {}

    void setLookAheadDistance(double lookAheadDistance)
    {
        lookAheadDistance_ = lookAheadDistance;
    }

    Velocity<> step(Pose<> currentPose, Velocity<> command)
    {
        double slope = atan2(referencePose_.y - currentPose.y,
            referencePose_.x - currentPose.x);

        double alpha = Math::normalizeAngleRad(currentPose.theta - slope);

        double angular = 2 * sin(alpha) / lookAheadDistance_;
        if (abs(abs(alpha) - M_PI) < 1e-8)
        {
            angular = Math::sgn(angular) * normalAngular_;
        }
        angular = saturate(angular, maxAngular_);

        double linear = command.linear;

        return Velocity<>(linear, angular);
    }

private:
    double lookAheadDistance_;
};

} // namespace srs

#endif // SIMPLECONTROLLER_HPP_
