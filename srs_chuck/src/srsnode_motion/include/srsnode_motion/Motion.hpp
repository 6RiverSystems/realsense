/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef MOTION_HPP_
#define MOTION_HPP_

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <srslib_framework/ros/tap/RosTapJoyAdapter.hpp>

#include <srsnode_motion/tap/goal_plan/RosTapGoalPlan.hpp>
#include <srsnode_motion/PositionEstimator.hpp>
#include <srsnode_motion/MotionController.hpp>

namespace srs {

class Motion
{
public:
    Motion(string nodeName);

    ~Motion()
    {
        disconnectAllTaps();
    }

    void reset();
    void run();

private:
    constexpr static unsigned int REFRESH_RATE_HZ = 50;

    void connectAllTaps();

    void disconnectAllTaps();

    void publishInformation();

    void scanTapsForData();

    bool commandUpdated_;
    Velocity<> currentCommand_;
    Pose<> currentPose_;
    ros::Time currentTime_;

    MotionController motionController_;

    PositionEstimator positionEstimator_;
    ros::Time previousTime_;
    ros::Publisher pubCmdVel_;
    ros::Publisher pubOdom_;
    ros::Publisher pubPose_;

    ros::NodeHandle rosNodeHandle_;
    tf::TransformBroadcaster rosTfBroadcaster_;

    RosTapGoalPlan tapPlan_;
    RosTapJoyAdapter<> tapJoyAdapter_;
    RosTapBrainStemStatus tapBrainStemStatus_;
    RosTapOdometry tapOdometry_;
    RosTapInitialPose tapInitialPose_;
};

} // namespace srs

#endif  // MOTION_HPP_
