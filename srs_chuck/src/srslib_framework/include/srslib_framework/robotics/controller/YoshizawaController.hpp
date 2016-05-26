/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef YOSHIZAWACONTROLLER_HPP_
#define YOSHIZAWACONTROLLER_HPP_

#include <opencv2/opencv.hpp>

#include <srslib_framework/math/PoseMath.hpp>
#include <srslib_framework/robotics/Pose.hpp>
#include <srslib_framework/robotics/Velocity.hpp>

namespace srs {

class YoshizawaController
{
public:
    YoshizawaController(double Kv, double Kw) :
        Kv_(Kv),
        Kw_(Kw)
    {}

    ~YoshizawaController()
    {}

    Velocity<> step(Pose<> currentPose, Velocity<> command)
    {
        // [ex, ey] = calculateError(o.rPose, cPose);
        double c = cos(currentPose.theta);
        double s = sin(currentPose.theta);
        double deltaX = referencePose_.x - currentPose.x;
        double deltaY = referencePose_.y - currentPose.y;

        double ex = deltaX * c + deltaY * s;
        double ey = -deltaX * s + deltaY * c;

        // vb = o.Kv;
        // linear = max(commandTwist.linear, vb);
        double linear = max(Kv_, command.linear);

        // A = sign(ex) * ey / ex^2;
        // wb = 2 * A * o.Kw;
        // angular = wb;
        double angular = 2 * Kw_ * Math::sgn(ex) * ey / (ex * ex);

        return Velocity<>(linear, angular);
    }

    void setReference(Pose<> referencePose)
    {
        referencePose_ = referencePose;
    }

private:
    double Kv_;
    double Kw_;

    Pose<> referencePose_;
};

} // namespace srs

#endif // YOSHIZAWACONTROLLER_HPP_