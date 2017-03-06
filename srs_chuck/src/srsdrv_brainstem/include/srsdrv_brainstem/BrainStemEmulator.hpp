/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <srslib_framework/platform/observer/Observer.hpp>
#include <srslib_framework/robotics/Pose.hpp>
#include <srslib_framework/robotics/Velocity.hpp>
#include <srslib_framework/ros/channel/ChannelBrainstemOdometryPose.hpp>
#include <srslib_framework/ros/tap/TapOdometryCmd_Velocity.hpp>
#include <srslib_framework/ros/tap/TapInitialPose.hpp>
#include <srslib_framework/ros/tap/subscriber/Subscriber.hpp>

namespace srs
{

class BrainStemEmulator :
    public Observer<Subscriber<geometry_msgs::PoseWithCovarianceStamped>>
{
public:
    BrainStemEmulator();
    virtual ~BrainStemEmulator();

private:
    static constexpr auto EMULATION_TICK_HZ = 150;

    Pose<> estimatePose(const Pose<>& pose0, float dfTimeDelta, const Velocity<>& velocity);

    void notified(Subscriber<geometry_msgs::PoseWithCovarianceStamped>* subject);

    void publishOdometry();
    void publishTransform();

    void tickEmulation(const ros::TimerEvent& event);

    tf::Transform brainstemTransform_;

    ChannelBrainstemOdometryPose channelOdometryPose_;
    Pose<> currentPose_;
    ros::Time currentTime_;
    tf::Transform currentTransform_;
    Velocity<> currentVelocity_;

    tf::TransformBroadcaster tfBroadcaster_;
    ros::Timer tickTimer_;
    TapOdometryCmd_Velocity tapCmdVelocity_;
    TapInitialPose tapInitialPose_;
};

} // namespace srs
