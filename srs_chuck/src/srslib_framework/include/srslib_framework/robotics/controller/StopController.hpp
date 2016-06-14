/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef STOPCONTROLLER_HPP_
#define STOPCONTROLLER_HPP_

#include <srslib_framework/math/AngleMath.hpp>
#include <srslib_framework/math/BasicMath.hpp>

#include <srslib_framework/robotics/controller/BaseController.hpp>

namespace srs {

class StopController: public BaseController
{
public:
    StopController() :
        BaseController(),
        emergency_(false)
    {}

    ~StopController()
    {}

    void emergency()
    {
        emergency_ = true;
    }

    void normal()
    {
        emergency_ = false;
    }

    void reset()
    {
        BaseController::reset();

        emergency_ = false;
    }

protected:
    void stepController(double dT, Pose<> currentPose, Odometry<> currentOdometry)
    {
    }

private:
    bool emergency_;
};

} // namespace srs

#endif // STOPCONTROLLER_HPP_
