/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef CMUPATHFOLLOWER_HPP_
#define CMUPATHFOLLOWER_HPP_

#include <srslib_framework/math/AngleMath.hpp>
#include <srslib_framework/math/BasicMath.hpp>

#include <srslib_framework/robotics/controller/BaseController.hpp>

namespace srs {

/**
 * J. M. Snider, "Automatic Steering Methods for Autonomous
 * Automobile Path Tracking", Robotics Institute, Carnegie Mellon
 * University, Pittsburgh, PA, USA, Tech. Report CMU-RI-TR-09-08, Feb. 2009
 */
class CMUPathFollower: public BaseController
{
public:
    CMUPathFollower(double Kv, double Kw) :
        BaseController(Kv, Kw),
        lookAheadDistance_(0.5)
    {}

    ~CMUPathFollower()
    {}

    void setLookAheadDistance(double lookAheadDistance)
    {
        lookAheadDistance_ = lookAheadDistance;
    }

    Velocity<> step(Pose<> currentPose, Velocity<> command)
    {
        double linear = Kv_ * command.linear;
        linear = BasicMath::saturate<double>(linear, maxLinear_, -maxLinear_);

        double slope = atan2(referencePose_.y - currentPose.y, referencePose_.x - currentPose.x);
        double alpha = AngleMath::normalizeAngleRad(slope - currentPose.theta);

        double angular = Kw_ * 2 * sin(alpha) / lookAheadDistance_;
        angular = BasicMath::saturate<double>(angular, maxAngular_, -maxAngular_);

        return Velocity<>(linear, angular);
    }

private:
    double lookAheadDistance_;
};

} // namespace srs

#endif // CMUPATHFOLLOWER_HPP_
