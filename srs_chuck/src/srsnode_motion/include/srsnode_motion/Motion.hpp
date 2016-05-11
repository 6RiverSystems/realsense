/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef MOTION_HPP_
#define MOTION_HPP_

#include <srslib_framework/robotics/Velocity.hpp>

#include <tf/transform_broadcaster.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>

#include <srslib_framework/ros/RosTap.hpp>
#include <srslib_framework/ros/tap/RosTapCmdVel.hpp>
#include <srslib_framework/filter/ukf/UnscentedKalmanFilter.hpp>

#include <srsnode_motion/Configuration.hpp>
#include <srsnode_motion/Robot.hpp>
#include <srsnode_motion/StatePe.hpp>

#include <srsnode_motion/tap/odometry/RosTapOdometry.hpp>
#include <srsnode_motion/tap/brain_stem_status/RosTapBrainStemStatus.hpp>
#include <srsnode_motion/tap/initial_pose/RosTapInitialPose.hpp>


#include <srsnode_motion/tap/goal_plan/RosTapGoalPlan.hpp>

namespace srs {

class Motion
{
public:
    Motion(string nodeName);

    ~Motion()
    {
        disconnectAllTaps();
    }

    void run();

private:
    typedef pair<Pose<>, Velocity<>> MilestoneType;

    constexpr static unsigned int REFRESH_RATE_HZ = 50;
    constexpr static double ALPHA = 1.0;
    constexpr static double BETA = 0.0;

    void disconnectAllTaps();

    void publishInformation();

    void scanTapsForData();
    void stepMotionController(double dT);
    void stepUkf(double dT);

    vector<MilestoneType> trajectory_;

    ros::NodeHandle rosNodeHandle_;

    bool commandUpdated_;
    CmdVelocity<> currentCommand_;
    cv::Mat currentCovariance_;
    StatePe<> currentState_;
    ros::Time currentTime_;

    double executionTime_;

    vector<MilestoneType>::iterator nextScheduled_;
    double nextScheduledTime_;

    ros::Time previousTime_;
    ros::Publisher pubCmdVel_;
    ros::Publisher pubOdom_;
    tf::TransformBroadcaster rosTfBroadcaster_;

    Robot<> robot_;

    RosTapBrainStemStatus tapBrainStemStatus_;
    RosTapCmdVel<> tapCmdVel_;
    RosTapOdometry tapOdometry_;
    RosTapInitialPose tapInitialPose_;
    RosTapGoalPlan tapPlan_;

    UnscentedKalmanFilter<STATIC_UKF_STATE_VECTOR_SIZE, STATIC_UKF_COMMAND_VECTOR_SIZE> ukf_;

    // TODO: Remove these variables
    double previousTimeNs_;
    double previousTimeS_;
    double currentTimeNs_;
    double currentTimeS_;
};

} // namespace srs

#endif  // MOTION_HPP_
