/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef MOTIONCONTROLLER_HPP_
#define MOTIONCONTROLLER_HPP_

#include <vector>
using namespace std;

#include <srslib_framework/planning/pathplanning/Trajectory.hpp>
#include <srslib_framework/robotics/Pose.hpp>
#include <srslib_framework/robotics/Velocity.hpp>
#include <srslib_framework/ros/RosTap.hpp>

#include <srslib_framework/robotics/controller/YoshizawaController.hpp>

namespace srs {

class MotionController
{
public:
    MotionController(unsigned int lookAhead = 50);

    ~MotionController()
    {}

    Velocity<> getExecutingCommand()
    {
        newCommandAvailable_ = false;
        return executingCommand_;
    }

    bool isGoalReached()
    {
        return goalReached_;
    }

    bool isMoving()
    {
        return executionTime_ > -1;
    }

    bool newCommandAvailable()
    {
        return newCommandAvailable_;
    }

    void reset();
    void run(double dT, Pose<> robotPose);

    void setLookAhead(unsigned int newValue)
    {
// ########
//        lookAhead_ = newValue;
//        if (isMoving())
//        {
//            stop();
//        }
    }

    void setTrajectory(Trajectory::TrajectoryType& trajectory);
    void stop(double stopDistance = 0);

private:
    void determineNextReferencePose();

    void sendVelocityCommand(Velocity<> command);

    // TODO: find a better place for it
    bool similarVelocities(const Velocity<>& lhv, const Velocity<>& rhv)
    {
        // The two velocities to be similar must:
        // - difference in linear velocity must be less than 0.01 m/s
        // - difference in angular velocity must be less than 0.1 deg/s
        return abs(lhv.linear - rhv.linear) < 0.01 && abs(lhv.angular - rhv.angular) < 0.002;
    }

    void stepLLController(Pose<> robotPose);

    Trajectory::TrajectoryType currentTrajectory_;

    Velocity<> executingCommand_;
    double executionTime_;

    Pose<> goal_;
    bool goalReached_;

    unsigned int lookAhead_;
    YoshizawaController lowLevelController_;

    bool newCommandAvailable_;
    vector<Trajectory::MilestoneType>::iterator nextScheduled_;
    double nextScheduledTime_;

    Pose<> referencePose_;
    int referenceIndex_;
};

} // namespace srs

#endif  // MOTIONCONTROLLER_HPP_
