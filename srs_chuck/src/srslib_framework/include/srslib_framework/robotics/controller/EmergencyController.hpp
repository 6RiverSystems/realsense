/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef EMERGENCYCONTROLLER_HPP_
#define EMERGENCYCONTROLLER_HPP_

#include <srslib_framework/robotics/Trajectory.hpp>
#include <srslib_framework/robotics/controller/BaseController.hpp>

namespace srs {

class EmergencyController: public BaseController
{
public:
    EmergencyController() :
        BaseController("EMERGENCY CONTROLLER")
    {}

    ~EmergencyController()
    {}

    void reset()
    {
        BaseController::reset();

        userCommand_ = Velocity<>::ZERO;
    }

    void setRobotProfile(RobotProfile robot)
    {
        BaseController::setRobotProfile(robot, robot.emergencyKv, robot.emergencyKw);
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
        setGoalReached(false);

        double linear = userCommand_.linear * robot_.emergencyMaxLinearVelocity;
        double angular = userCommand_.angular * robot_.emergencyMaxAngularVelocity;

        // If the robot is moving backward and at the same time
        // rotating, invert the direction of rotation. No transformation
        // is performed if the robot is rotating in place (linear = 0)
        if (!BasicMath::equal(userCommand_.linear, 0.0, 0.001))
        {
            angular *= BasicMath::sgn<double>(userCommand_.linear);
        }

        // Send the command for execution
        executeCommand(linear, angular);
    }

private:
    Velocity<> userCommand_;
};

} // namespace srs

#endif // EMERGENCYCONTROLLER_HPP_
