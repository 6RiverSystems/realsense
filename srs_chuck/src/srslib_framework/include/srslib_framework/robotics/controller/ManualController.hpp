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
        BaseController("MANUAL CONTROLLER")
    {}

    ~ManualController()
    {}

    void reset()
    {
        BaseController::reset();

        userCommand_ = Velocity<>();
    }

    void setUserCommand(Velocity<> userCommand)
    {
        userCommand_ = userCommand;
    }

protected:
    void stepController(double dT, Pose<> currentPose, Odometry<> currentOdometry)
    {
        // The controller never completes its goal. In order to complete its
        // goal, it must be canceled
        goalReached_ = false;

        double linear = BasicMath::saturate<double>(
            robot_.ratioManualLinear * userCommand_.linear * robot_.travelLinearVelocity,
            robot_.maxLinearVelocity, -robot_.maxLinearVelocity);

        double angular = BasicMath::saturate<double>(
            robot_.ratioManualAngular * userCommand_.angular * robot_.travelAngularVelocity,
            robot_.maxAngularVelocity, -robot_.maxAngularVelocity);

        // If the robot is moving backward and at the same time
        // rotating, invert the direction of rotation. No transformation
        // is performed if the robot is rotating in place (linear = 0)
        if (!BasicMath::equal(userCommand_.linear, 0.0, 0.001))
        {
            angular *= BasicMath::sgn<double>(userCommand_.linear);
        }

        // Send the command for execution
        executeCommand(Velocity<>(linear, angular));
    }

private:
    Velocity<> userCommand_;
};

} // namespace srs

#endif // MANUALCONTROLLER_HPP_
