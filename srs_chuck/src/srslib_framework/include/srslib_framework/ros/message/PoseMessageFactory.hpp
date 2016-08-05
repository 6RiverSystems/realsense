/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef POSEMESSAGEFACTORY_HPP_
#define POSEMESSAGEFACTORY_HPP_

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include <srslib_framework/math/TimeMath.hpp>
#include <srslib_framework/MsgPose.h>

namespace srs {

struct PoseMessageFactory
{
    /**
     * @brief Convert a MsgPose type into a Pose.
     *
     * @param message MsgPose to convert
     *
     * @return Pose generated from the specified MsgPose
     */
    static Pose<> msg2Pose(srslib_framework::MsgPose message)
    {
        Pose<> pose;

        pose.arrivalTime = TimeMath::time2number(message.header.stamp);
        pose.x = message.x;
        pose.y = message.y;
        pose.theta = AngleMath::deg2rad<double>(message.theta);

        return pose;
    }

    /**
     * @brief Convert a MsgPoseConstPtr type into a Pose.
     *
     * @param message MsgPose to convert
     *
     * @return Pose generated from the specified MsgPose
     */
    static Pose<> msg2Pose(srslib_framework::MsgPoseConstPtr message)
    {
        return PoseMessageFactory::msg2Pose(*message);
    }

    /**
     * @brief Convert a Pose type into a MsgPose.
     *
     * @param pose Pose to convert
     * @param timestamp ROS time stamp for the message
     *
     * @return MsgPose generated from the specified Pose
     */
    static srslib_framework::MsgPose pose2Msg(Pose<> pose, ros::Time timestamp = ros::Time::now())
    {
        srslib_framework::MsgPose msgPose;
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
    static geometry_msgs::PoseStamped pose2PoseStamped(Pose<> pose,
        ros::Time timestamp = ros::Time::now())
    {
        geometry_msgs::PoseStamped poseStamped;
        poseStamped.header.stamp = timestamp;

        tf::Quaternion quaternion = tf::createQuaternionFromYaw(pose.theta);

        poseStamped.pose.position.x = pose.x;
        poseStamped.pose.position.y = pose.y;
        poseStamped.pose.position.z = 0.0;
        poseStamped.pose.orientation.x = quaternion.x();
        poseStamped.pose.orientation.y = quaternion.y();
        poseStamped.pose.orientation.z = quaternion.z();
        poseStamped.pose.orientation.w = quaternion.w();

        return poseStamped;
    }

    /**
     * @brief Convert a PoseStamped type into a Pose.
     *
     * @param message PoseStamped to convert
     *
     * @return Pose generated from the specified PoseStamped
     */
    static Pose<> poseStamped2Pose(geometry_msgs::PoseStamped message)
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
    static Pose<> poseStamped2Pose(geometry_msgs::PoseStampedConstPtr message)
    {
        return PoseMessageFactory::poseStamped2Pose(*message);
    }
};

} // namespace srs

#endif // POSEMESSAGEFACTORY_HPP_
