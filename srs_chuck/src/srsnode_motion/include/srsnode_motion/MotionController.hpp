/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef MOTIONCONTROLLER_HPP_
#define MOTIONCONTROLLER_HPP_

#include <srslib_framework/robotics/Pose.hpp>
#include <srslib_framework/robotics/Velocity.hpp>
#include <srslib_framework/ros/RosTap.hpp>

namespace srs {

class MotionController
{
public:
    MotionController();

    ~MotionController()
    {}

    Velocity<> getExecutingCommand()
    {
        newCommandAvailable_ = false;
        return executingCommand_;
    }

    bool newCommandAvailable()
    {
        return newCommandAvailable_;
    }

    void reset();
    void run(double dT, Pose<> robotPose, Velocity<>* command);

private:
    typedef pair<Pose<>, Velocity<>> MilestoneType;

    double executionTime_;

    bool newCommandAvailable_;
    vector<MilestoneType>::iterator nextScheduled_;
    double nextScheduledTime_;

    vector<MilestoneType> trajectory_;

    Velocity<> executingCommand_;
};

} // namespace srs

#endif  // MOTIONCONTROLLER_HPP_
