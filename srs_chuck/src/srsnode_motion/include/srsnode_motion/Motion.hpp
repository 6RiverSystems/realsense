/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef MOTION_HPP_
#define MOTION_HPP_

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <tf/transform_broadcaster.h>

#include <srsnode_motion/MotionConfig.h>
using namespace srsnode_motion;

#include <dynamic_reconfigure/server.h>

#include <srslib_framework/graph/grid2d/Grid2d.hpp>
#include <srslib_framework/planning/pathplanning/TrajectoryGenerator.hpp>
#include <srslib_framework/ros/tap/RosTapBrainStem.hpp>
#include <srslib_framework/ros/tap/RosTapInternal_Goal.hpp>
#include <srslib_framework/ros/tap/RosTapInternal_InitialPose.hpp>
#include <srslib_framework/ros/tap/RosTapJoyAdapter.hpp>
#include <srslib_framework/ros/tap/RosTapMap.hpp>
#include <srslib_framework/ros/tap/RosTapOdometry.hpp>
#include <srslib_framework/ros/service/RosTriggerShutdown.hpp>
#include <srslib_framework/ros/service/RosTriggerStop.hpp>
#include <srslib_framework/search/AStar.hpp>

#include <srsnode_motion/PositionEstimator.hpp>
#include <srsnode_motion/MotionController.hpp>
#include <srsnode_motion/tap/aps/RosTapAps.hpp>

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
    constexpr static double REFRESH_RATE_HZ = 100;

    void connectAllTaps();

    void disconnectAllTaps();

    void evaluateTriggers();
    void executePlanToGoal(Pose<> goal);

    void onConfigChange(MotionConfig& config, uint32_t level);

    void publishArrived();
    void publishGoal();
    void publishOdometry();
    void publishPing();

    void reset(Pose<> pose0);

    void scanTapsForData();
    void stepNode();

    AStar<Grid2d> algorithm_;

    MotionConfig configuration_;
    dynamic_reconfigure::Server<MotionConfig> configServer_;
    ros::Time currentTime_;

    bool firstLocalization_;

    bool isJoystickLatched_;
    bool isOdometryAvailable_;

    MotionController motionController_;

    PositionEstimator positionEstimator_;
    ros::Time previousTime_;
    ros::Publisher pubOdometry_;
    ros::Publisher pubStatusGoalPlan_;
    ros::Publisher pubStatusGoalGoal_;
    ros::Publisher pubStatusGoalArrived_;
    ros::Publisher pubPing_;

    ros::NodeHandle rosNodeHandle_;
    tf::TransformBroadcaster rosTfBroadcaster_;
    Chuck robot_;

    double simulatedT_;

    RosTapAps tapAps_;
    RosTapBrainStem tapBrainStem_;
    RosTapInternal_InitialPose tapInitialPose_;
    RosTapInternal_Goal tapInternalGoal_;
    RosTapJoyAdapter tapJoyAdapter_;
    RosTapMap tapMap_;
    RosTapOdometry tapOdometry_;
    RosTriggerStop triggerStop_;
    RosTriggerShutdown triggerShutdown_;
};

} // namespace srs

#endif  // MOTION_HPP_
