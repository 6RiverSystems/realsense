/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef BASECONTROLLER_HPP_
#define BASECONTROLLER_HPP_

#include <srslib_framework/math/BasicMath.hpp>
#include <srslib_framework/robotics/Pose.hpp>
#include <srslib_framework/robotics/Velocity.hpp>

namespace srs {

class BaseController
{
public:
    BaseController(double Kv, double Kw) :
        Kv_(Kv),
        Kw_(Kw),
        maxAngular_(0.0),
        maxLinear_(0.0),
        travelAngular_(0.0),
        travelLinear_(0.0)
    {}

    virtual ~BaseController()
    {}

    virtual Velocity<> step(Pose<> currentPose, Velocity<> command) = 0;

    void setMaxAngularVelocity(double value)
    {
        maxAngular_ = value;
    }

    void setMaxLinearVelocity(double value)
    {
        maxLinear_ = value;
    }

    void setTravelAngularVelocity(double value)
    {
        travelAngular_ = value;
    }

    void setTravelLinearVelocity(double value)
    {
        travelLinear_ = value;
    }

    void setReference(Pose<> referencePose)
    {
        referencePose_ = referencePose;
    }

protected:
    double Kv_;
    double Kw_;

    double maxAngular_;
    double maxLinear_;

    double travelAngular_;
    double travelLinear_;

    Pose<> referencePose_;
};

} // namespace srs

#endif // BASECONTROLLER_HPP_
