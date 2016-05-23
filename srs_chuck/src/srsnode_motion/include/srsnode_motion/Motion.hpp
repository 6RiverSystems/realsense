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

#include <srsnode_motion/DynamicConfig.h>
using namespace srsnode_motion;

#include <dynamic_reconfigure/server.h>

#include <srslib_framework/planning/pathplanning/Trajectory.hpp>
#include <srslib_framework/ros/tap/RosTapJoyAdapter.hpp>
#include <srslib_framework/ros/service/RosTriggerShutdown.hpp>
#include <srslib_framework/ros/service/RosTriggerStop.hpp>

#include <srsnode_motion/tap/aps/RosTapAps.hpp>
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

    void evaluateTriggers();

    void onConfigChange(DynamicConfig& config, uint32_t level);

    void publishInformation();

    void scanTapsForData();
    void sendVelocityCommand(Velocity<> command);

    bool commandUpdated_;
    DynamicConfig configuration_;
    dynamic_reconfigure::Server<DynamicConfig> configServer_;
    Velocity<> currentCommand_;
    ros::Time currentTime_;

    MotionController motionController_;

    PositionEstimator positionEstimator_;
    ros::Time previousTime_;
    ros::Publisher pubCmdVel_;
    ros::Publisher pubOdometry_;

    ros::NodeHandle rosNodeHandle_;
    tf::TransformBroadcaster rosTfBroadcaster_;
    Chuck robot_;

    RosTapAps tapAps_;
    RosTapGoalPlan tapPlan_;
    RosTapJoyAdapter<> tapJoyAdapter_;
    RosTapBrainStemStatus tapBrainStemStatus_;
    RosTapOdometry tapOdometry_;
    RosTapInitialPose tapInitialPose_; // TODO: This should be a command (trigger) passed down by Executive
    Trajectory trajectoryConverter_;
    RosTriggerStop triggerStop_;
    RosTriggerShutdown triggerShutdown_;

};

} // namespace srs

#endif  // MOTION_HPP_
