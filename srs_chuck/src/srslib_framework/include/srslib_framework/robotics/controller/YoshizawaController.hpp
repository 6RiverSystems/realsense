/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef YOSHIZAWACONTROLLER_HPP_
#define YOSHIZAWACONTROLLER_HPP_

#include <srslib_framework/math/BasicMath.hpp>
#include <srslib_framework/robotics/controller/BaseController.hpp>

namespace srs {

class YoshizawaController : public BaseController
{
public:
    YoshizawaController() :
        BaseController()
    {}

    ~YoshizawaController()
    {}

    Velocity<> stepController(Pose<> currentPose, Pose<> desiredPose, Velocity<> command)
    {
        // [ex, ey] = calculateError(o.rPose, cPose);
        double c = cos(currentPose.theta);
        double s = sin(currentPose.theta);
        double deltaX = desiredPose.x - currentPose.x;
        double deltaY = desiredPose.y - currentPose.y;

        double ex = deltaX * c + deltaY * s;
        double ey = -deltaX * s + deltaY * c;

        // vb = o.Kv;
        // linear = max(commandTwist.linear, vb);
        double linear = max(Kv_, command.linear);

        // A = sign(ex) * ey / ex^2;
        // wb = 2 * A * o.Kw;
        // angular = wb;
        double angular = 2 * Kw_ * BasicMath::sgn<double>(ex) * ey / (ex * ex);

        return Velocity<>(linear, angular);
    }
};

} // namespace srs

#endif // YOSHIZAWACONTROLLER_HPP_
