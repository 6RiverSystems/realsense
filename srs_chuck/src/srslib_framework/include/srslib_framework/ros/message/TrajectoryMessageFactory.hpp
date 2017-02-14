/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <nav_msgs/Path.h>

#include <srslib_framework/Pose.h>
#include <srslib_framework/planning/pathplanning/trajectory/Trajectory.hpp>
#include <srslib_framework/chuck/ChuckTransforms.hpp>

namespace srs {

struct TrajectoryMessageFactory
{
    /**
     * @brief Convert a Trajectory into a nav_msgs::Path.
     *
     * @param trajectory Trajectory to convert
     * @param timestamp ROS time stamp for the message
     *
     * @return nav_msgs::Path generated from the specified Trajectory
     */
    static nav_msgs::Path trajectory2RosPathMsg(const Trajectory<>& trajectory,
        const ros::Time timestamp = ros::Time::now())
    {
        nav_msgs::Path messagePath;
        messagePath.header.frame_id = ChuckTransforms::MAP;
        messagePath.header.stamp = timestamp;

        vector<geometry_msgs::PoseStamped> planPoses;

        for (auto waypoint : trajectory)
        {
            geometry_msgs::PoseStamped poseStamped;
            tf::Quaternion quaternion = tf::createQuaternionFromYaw(waypoint.first.theta);

            poseStamped.pose.position.x = waypoint.first.x;
            poseStamped.pose.position.y = waypoint.first.y;
            poseStamped.pose.position.z = 0.0;
            poseStamped.pose.orientation.x = quaternion.x();
            poseStamped.pose.orientation.y = quaternion.y();
            poseStamped.pose.orientation.z = quaternion.z();
            poseStamped.pose.orientation.w = quaternion.w();

            planPoses.push_back(poseStamped);
        }

        messagePath.poses = planPoses;

        return messagePath;
    }
};

} // namespace srs
