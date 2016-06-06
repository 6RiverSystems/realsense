/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef BASECONTROLLER_HPP_
#define BASECONTROLLER_HPP_

#include <srslib_framework/math/PoseMath.hpp>
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
        normalAngular_(0.0),
        normalLinear_(0.0)
    {}

    virtual ~BaseController()
    {}

    virtual Velocity<> step(Pose<> currentPose, Velocity<> command) = 0;

    void setMaxAngular(double value)
    {
        maxAngular_ = value;
    }

    void setMaxLinear(double value)
    {
        maxLinear_ = value;
    }

    void setNormalAngular(double value)
    {
        normalAngular_ = value;
    }

    void setNormalLinear(double value)
    {
        normalLinear_ = value;
    }

    void setReference(Pose<> referencePose)
    {
        referencePose_ = referencePose;
    }

protected:
    double saturate(double value, double max)
    {
        return abs(value) > max ? Math::sgn(value) * max : value;
    }

    double Kv_;
    double Kw_;

    double maxAngular_;
    double maxLinear_;

    double normalAngular_;
    double normalLinear_;

    Pose<> referencePose_;
};

} // namespace srs

#endif // BASECONTROLLER_HPP_
