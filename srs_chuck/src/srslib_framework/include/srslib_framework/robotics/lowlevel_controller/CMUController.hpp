/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef CMUCONTROLLER_HPP_
#define CMUCONTROLLER_HPP_

#include <srslib_framework/math/Math.hpp>

#include <srslib_framework/robotics/lowlevel_controller/BaseController.hpp>

namespace srs {

class CMUController: public BaseController
{
public:
    CMUController(double Kv, double Kw) :
        BaseController(Kv, Kw),
        lookAheadDistance_(1.5)
    {}

    ~CMUController()
    {}

    void setLookAheadDistance(double lookAheadDistance)
    {
        lookAheadDistance_ = lookAheadDistance;
    }

    Velocity<> step(Pose<> currentPose, Velocity<> command)
    {
        double slope = atan2(referencePose_.y - currentPose.y, referencePose_.x - currentPose.x);
        double alpha = Math::normalizeAngleRad(slope - currentPose.theta);

        double angular = 2 * sin(alpha) / lookAheadDistance_;
        angular = saturate(angular, maxAngular_);

        double linear = command.linear;
        linear = saturate(linear, maxLinear_);

        return Velocity<>(linear, angular);
    }

private:
    double lookAheadDistance_;
};

} // namespace srs

#endif // CMUCONTROLLER_HPP_
