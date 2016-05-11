/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef MOTION_HPP_
#define MOTION_HPP_

#include <srslib_framework/robotics/Velocity.hpp>

#include <srsnode_motion/tap/RosTapGoalPlan.hpp>

namespace srs {

class Motion
{
public:
    Motion(string nodeName);

    ~Motion()
    {
    }

    void run();

private:
    constexpr static unsigned int REFRESH_RATE_HZ = 20;

    vector<Velocity<>> cmdVel_;

    ros::Publisher pubCmdVel_;

    ros::NodeHandle rosNodeHandle_;
    double executionTime_;
    int nextScheduled_;
    RosTapGoalPlan tapPlan_;
};

} // namespace srs

#endif  // MOTION_HPP_
