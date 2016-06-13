/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef MANUALCONTROLLER_HPP_
#define MANUALCONTROLLER_HPP_

#include <srslib_framework/robotics/Trajectory.hpp>
#include <srslib_framework/robotics/controller/BaseController.hpp>

namespace srs {

class ManualController: public BaseController
{
public:
    ManualController() :
        BaseController()
    {}

    ~ManualController()
    {}

    void reset();

    void setUserCommand(Velocity<> userCommand)
    {
        userCommand_ = userCommand;
    }

protected:
    void stepController(Pose<> currentPose, Odometry<> currentOdometry);

private:
    Velocity<> userCommand_;
};

} // namespace srs

#endif // MANUALCONTROLLER_HPP_
