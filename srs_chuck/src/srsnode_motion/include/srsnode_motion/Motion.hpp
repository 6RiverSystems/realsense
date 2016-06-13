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

#include <srslib_framework/planning/pathplanning/TrajectoryGenerator.hpp>

#include <srslib_framework/ros/tap/RosTapJoyAdapter.hpp>
#include <srslib_framework/ros/tap/RosTapMap.hpp>
#include <srslib_framework/ros/service/RosTriggerShutdown.hpp>
#include <srslib_framework/ros/service/RosTriggerStop.hpp>

#include <srsnode_motion/tap/aps/RosTapAps.hpp>

//#include <srsnode_motion/tap/goal_plan/RosTapGoalPlan.hpp>
#include <srsnode_motion/PositionEstimator.hpp>
#include <srsnode_motion/MotionController.hpp>

#include <srslib_framework/graph/grid2d/Grid2d.hpp>
#include <srslib_framework/search/AStar.hpp>

#include <srsnode_motion/tap/RosTapCmd_Goal.hpp>

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
    constexpr static unsigned int REFRESH_RATE_HZ = 60;

    void connectAllTaps();

    void disconnectAllTaps();

    void evaluateTriggers();

    void onConfigChange(MotionConfig& config, uint32_t level);

    void publishInformation();

    void reset(Pose<> pose0);

    void scanTapsForData();
    void stepNode();

    AStar<Grid2d> algorithm_; // TODO: Remove

    MotionConfig configuration_;
    dynamic_reconfigure::Server<MotionConfig> configServer_;
    Velocity<> currentCommand_;
    ros::Time currentTime_;

    bool firstLocalization_;

    MotionController motionController_;

    PositionEstimator positionEstimator_;
    ros::Time previousTime_;
    ros::Publisher pubOdometry_;

    ros::NodeHandle rosNodeHandle_;
    tf::TransformBroadcaster rosTfBroadcaster_;
    Chuck robot_;

    RosTapAps tapAps_;
    // RosTapGoalPlan tapPlan_;
    RosTapJoyAdapter tapJoyAdapter_;
    RosTapBrainStem tapBrainStem_;
    RosTapOdometry tapOdometry_;
    RosTapInitialPose tapInitialPose_; // TODO: This should be a command (trigger) passed down by Executive
    RosTriggerStop triggerStop_;
    RosTriggerShutdown triggerShutdown_;

    // TODO: remove this tap from Motion. It should be in Executive
    RosTapCmd_Goal tapCmdGoal_;
    RosTapMap tapMap_;
    ros::Publisher pubGoalPlan_;
    ros::Publisher pubGoalGoal_;

    void executePlanToGoal(Pose<> goal);
    void publishGoal();

    double simulatedT_;
};

} // namespace srs

#endif  // MOTION_HPP_
