/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <srslib_framework/Pose.h>

#include <srslib_framework/math/TimeMath.hpp>
#include <srslib_framework/robotics/Pose.hpp>

namespace srs {

struct PoseMessageFactory
{
    /**
     * @brief Convert a Pose type into a Pose.
     *
     * @param message Pose to convert
     *
     * @return Pose generated from the specified Pose
     */
    static Pose<> msg2Pose(const srslib_framework::Pose& message)
    {
        Pose<> pose;

        pose.arrivalTime = TimeMath::time2number(message.header.stamp);
        pose.x = message.x;
        pose.y = message.y;
        pose.theta = AngleMath::deg2Rad<double>(message.theta);

        return pose;
    }

    /**
     * @brief Convert a Pose pointer type into a Pose.
     *
     * @param message Pose to convert
     *
     * @return Pose generated from the specified Pose
     */
    static Pose<> msg2Pose(srslib_framework::Pose::ConstPtr message)
    {
        return msg2Pose(*message);
    }

    /**
     * @brief Convert a ROS Odometry pointer type into a Pose.
     *
     * @param message Pose to convert
     *
     * @return Pose generated from the specified Pose
     */
    static Pose<> msg2Pose(nav_msgs::Odometry::ConstPtr message)
    {
        return msg2Pose(*message);
    }

    /**
     * @brief Convert a ROS Pose type into a Pose.
     *
     * @param message Pose to convert
     *
     * @return Pose generated from the specified Pose
     */
    static Pose<> msg2Pose(const geometry_msgs::Pose& message)
    {
        Pose<> pose;

        pose.arrivalTime = 0;
        pose.x = message.position.x;
        pose.y = message.position.y;
        pose.theta = tf::getYaw(message.orientation);

        return pose;
    }

    /**
     * @brief Convert a ROS Odometry type into a Pose.
     *
     * @param message Pose to convert
     *
     * @return Pose generated from the specified Pose
     */
    static Pose<> msg2Pose(const nav_msgs::Odometry& message)
    {
        Pose<> pose;

        pose.arrivalTime = TimeMath::time2number(message.header.stamp);
        pose.x = message.pose.pose.position.x;
        pose.y = message.pose.pose.position.y;
        pose.theta = tf::getYaw(message.pose.pose.orientation);

        return pose;
    }

    /**
     * @brief Convert a Pose type into a Pose.
     *
     * @param pose Pose to convert
     * @param timestamp ROS time stamp for the message
     *
     * @return Pose generated from the specified Pose
     */
    static srslib_framework::Pose pose2Msg(const Pose<> pose, ros::Time timestamp = ros::Time::now())
    {
        srslib_framework::Pose msgPose;
        msgPose.header.stamp = timestamp;

        msgPose.x = pose.x;
        msgPose.y = pose.y;
        msgPose.theta = pose.getThetaDegrees();

        return msgPose;
    }

    /**
     * @brief Convert a Pose type into a PoseStamped.
     *
     * @param pose Pose to convert
     * @param timestamp ROS time stamp for the message
     *
     * @return PoseStamped generated from the specified Pose
     */
    static geometry_msgs::PoseStamped pose2PoseStamped(const Pose<> pose,
        ros::Time timestamp = ros::Time::now())
    {
        geometry_msgs::PoseStamped msgPoseStamped;
        msgPoseStamped.header.stamp = timestamp;

        tf::Quaternion quaternion = tf::createQuaternionFromYaw(pose.theta);

        msgPoseStamped.pose.position.x = pose.x;
        msgPoseStamped.pose.position.y = pose.y;
        msgPoseStamped.pose.position.z = 0.0;
        msgPoseStamped.pose.orientation.x = quaternion.x();
        msgPoseStamped.pose.orientation.y = quaternion.y();
        msgPoseStamped.pose.orientation.z = quaternion.z();
        msgPoseStamped.pose.orientation.w = quaternion.w();

        return msgPoseStamped;
    }

    /**
     * @brief Convert a Pose type into a ROS Pose.
     *
     * @param pose Pose to convert
     *
     * @return ROS Pose generated from the specified Pose
     */
    static geometry_msgs::Pose pose2RosPose(const Pose<> pose)
    {
        geometry_msgs::Pose msgPose;

        msgPose.position.x = pose.x;
        msgPose.position.y = pose.y;
        msgPose.position.z = 0.0;

        tf::Quaternion quaternion = tf::createQuaternionFromYaw(pose.theta);

        msgPose.orientation.x = quaternion.x();
        msgPose.orientation.y = quaternion.y();
        msgPose.orientation.z = quaternion.z();
        msgPose.orientation.w = quaternion.w();

        return msgPose;
    }

    /**
     * @brief Convert a PoseStamped type into a Pose.
     *
     * @param message PoseStamped to convert
     *
     * @return Pose generated from the specified PoseStamped
     */
    static Pose<> poseStamped2Pose(const geometry_msgs::PoseStamped& message)
    {
        Pose<> pose;

        pose.arrivalTime = TimeMath::time2number(message.header.stamp);
        pose.x = message.pose.position.x;
        pose.y = message.pose.position.y;
        pose.theta = tf::getYaw(message.pose.orientation);

        return pose;
    }

    /**
     * @brief Convert a PoseStampedConstPtr type into a Pose.
     *
     * @param message PoseStampedConstPtr to convert
     *
     * @return Pose generated from the specified PoseStampedConstPtr
     */
    static Pose<> poseStamped2Pose(geometry_msgs::PoseStamped::ConstPtr message)
    {
        return poseStamped2Pose(*message);
    }

    /**
     * @brief Convert a PoseWithCovarianceStampedConstPtr type into a Pose.
     *
     * @param message PoseWithCovarianceStampedConstPtr to convert
     *
     * @return Pose generated from the specified PoseWithCovarianceStampedConstPtr
     */
    static Pose<> poseStampedWithCovariance2Pose(geometry_msgs::PoseWithCovarianceStampedConstPtr message)
    {
        Pose<> pose;

        pose.arrivalTime = TimeMath::time2number(message->header.stamp);
        pose.x = message->pose.pose.position.x;
        pose.y = message->pose.pose.position.y;
        pose.theta = tf::getYaw(message->pose.pose.orientation);

        return pose;
    }

    static tf::Transform pose2Transform(const Pose<> pose)
    {
        tf::Transform transform;
        transform.setOrigin(tf::Point(pose.x, pose.y, 0.0));
        transform.setRotation(tf::createQuaternionFromYaw(pose.theta));
        return transform;
    }

    static Pose<> transform2Pose(const tf::Transform transform)
    {
        tf::Vector3 location = transform.getOrigin();
        tf::Quaternion orientation = transform.getRotation();

        return Pose<>(location.getX(), location.getY(), tf::getYaw(orientation));
    }
};

} // namespace srs
