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
    CMUPathFollower() :
        BaseController(),
        lookAheadDistance_(0.5),
        travelRotationVelocity_(0.1)
    {}

    ~CMUPathFollower()
    {}

    void setLookAheadDistance(double lookAheadDistance)
    {
        lookAheadDistance_ = lookAheadDistance;
    }

    void setTravelRotationVelocity(double value)
    {
        travelRotationVelocity_ = value;
    }

    Velocity<> stepController(Pose<> currentPose, Pose<> desiredPose, Velocity<> command)
    {
        // Calculate the linear portion of the command
        double linear = Kv_ * command.linear;
        linear = BasicMath::saturate<double>(linear, maxLinear_, -maxLinear_);

        // Calculate the angular portion of the command
        double slope = atan2(desiredPose.y - currentPose.y, desiredPose.x - currentPose.x);
        double alpha = AngleMath::normalizeAngleRad(slope - currentPose.theta);

        double angular = Kw_ * 2 * sin(alpha) / lookAheadDistance_;

        // If the robot angle is greater than 45deg, use a different rotation velocity
        if (abs(alpha) > M_PI / 4)
        {
            angular = BasicMath::sgn(angular) * travelRotationVelocity_;
        }

        angular = BasicMath::saturate<double>(angular, maxAngular_, -maxAngular_);

        return Velocity<>(linear, angular);
    }

private:
    double lookAheadDistance_;
    double travelRotationVelocity_;
};

} // namespace srs

#endif // CMUPATHFOLLOWER_HPP_
