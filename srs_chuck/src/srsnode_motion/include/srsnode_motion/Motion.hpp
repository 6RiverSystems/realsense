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

#include <dynamic_reconfigure/server.h>

#include <srslib_framework/graph/grid2d/Grid2d.hpp>

#include <srslib_framework/ros/tap/RosTapBrainStem.hpp>
#include <srslib_framework/ros/tap/RosTapInternal_GoalSolution.hpp>
#include <srslib_framework/ros/tap/RosTapInternal_InitialPose.hpp>
#include <srslib_framework/ros/tap/RosTapJoyAdapter.hpp>
#include <srslib_framework/ros/tap/RosTapMap.hpp>
#include <srslib_framework/ros/service/RosServiceExecuteSolution.hpp>
#include <srslib_framework/ros/service/RosTriggerPause.hpp>
#include <srslib_framework/ros/service/RosTriggerShutdown.hpp>
#include <srslib_framework/ros/service/RosTriggerStop.hpp>

#include <srslib_framework/search/AStar.hpp>

#include <srsnode_motion/PositionEstimator.hpp>
#include <srsnode_motion/MotionController.hpp>
#include <srsnode_motion/tap/sensor_frame/aps/RosTapAps.hpp>
#include <srsnode_motion/tap/sensor_frame/RosTapSensorFrame.hpp>

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
    constexpr static double REFRESH_RATE_HZ = 90;
    constexpr static double PING_HZ = 10;
    constexpr static double MAX_ALLOWED_PING_DELAY = 0.5; // 50% of the duty cycle

    void connectAllTaps();

    void disconnectAllTaps();

    void evaluateTriggers();
    void executeSolution(Solution<GridSolutionItem> solution);

    void onConfigChange(MotionConfig& config, uint32_t level);

    void pingCallback(const ros::TimerEvent& event);
    void publishAccumulatedOdometry();
    void publishArrived();
    void publishGoalLanding();
    void publishImu();
    void publishOdometry();
    void publishLocalized();
    void publishPose();

    void reset(Pose<> pose0);

    void scanTapsForData();
    void stepNode();
    void stepEmulation();

    AStar<Grid2d> algorithm_;

    dynamic_reconfigure::Server<MotionConfig> configServer_;
    ros::Time currentTime_;

    bool isApsAvailable_;
    bool isEmulationEnabled_;
    bool isImuAvailable_;
    bool isJoystickLatched_;
    bool isOdometryAvailable_;
    bool isCustomActionEnabled_;

    MotionController motionController_;

    ros::Timer pingTimer_;
    PositionEstimator positionEstimator_;
    ros::Time previousTime_;
    ros::Publisher pubOdometry_;
    ros::Publisher pubPing_;
    ros::Publisher pubRobotAccOdometry_;
    ros::Publisher pubRobotPose_;
    ros::Publisher pubStatusGoalArrived_;
    ros::Publisher pubStatusGoalLanding_;
    ros::Publisher pubRobotLocalized_;

    ros::Publisher pubImu_;

    ros::NodeHandle rosNodeHandle_;
    tf::TransformBroadcaster rosTfBroadcaster_;
    Chuck robot_;

    double simulatedT_;

    RosTapAps tapAps_;
    RosTapBrainStem tapBrainStem_;
    RosTapSensorFrame tapSensorFrame_;
    RosTapInternal_GoalSolution tapInternalGoalSolution_;
    RosTapInternal_InitialPose tapInitialPose_;
    RosTapJoyAdapter tapJoyAdapter_;
    RosTapMap tapMap_;

    RosServiceExecuteSolution triggerExecuteSolution_;
    RosTriggerPause triggerPause_;
    RosTriggerStop triggerStop_;
    RosTriggerShutdown triggerShutdown_;
};

} // namespace srs

#endif  // MOTION_HPP_
