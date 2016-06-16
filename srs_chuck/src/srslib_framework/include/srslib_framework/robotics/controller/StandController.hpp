/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef STANDCONTROLLER_HPP_
#define STANDCONTROLLER_HPP_

#include <srslib_framework/math/AngleMath.hpp>
#include <srslib_framework/math/BasicMath.hpp>

#include <srslib_framework/robotics/controller/BaseController.hpp>

namespace srs {

class StandController: public BaseController
{
public:
    StandController() :
        BaseController("STAND CONTROLLER")
    {}

    ~StandController()
    {}

protected:
    void stepController(double dT, Pose<> currentPose, Odometry<> currentOdometry)
    {
        // The controller never completes its goal. In order to complete its
        // goal, it must be canceled
        goalReached_ = false;

        // Send the command for execution
        executeCommand(Velocity<>());
    }

private:
};

} // namespace srs

#endif // STANDCONTROLLER_HPP_
