/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/Pose.h>
#include <srslib_framework/MsgSolution.h>
#include <srslib_framework/MsgSolutionItem.h>

#include <srslib_framework/planning/pathplanning/grid2d/Grid2dSolutionItem.hpp>
#include <srslib_framework/ros/message/PoseMessageFactory.hpp>

namespace srs {

struct SolutionMessageFactory
{
    /**
     * @brief Convert a Grid2dSolutionItem type into a MsgSolutionItem.
     *
     * @param Grid2dSolutionItem Solution item to convert
     *
     * @return MsgSolutionItem generated from the specified Grid2dSolutionItem
     */
    static srslib_framework::MsgSolutionItem Grid2dSolutionItem2Msg(const Grid2dSolutionItem& Grid2dSolutionItem)
    {
        srslib_framework::MsgSolutionItem msgSolutionItem;

        msgSolutionItem.action = Grid2dSolutionItem.actionType;
        msgSolutionItem.fromPose = PoseMessageFactory::pose2Msg(Grid2dSolutionItem.fromPose);
        msgSolutionItem.toPose = PoseMessageFactory::pose2Msg(Grid2dSolutionItem.toPose);
        msgSolutionItem.cost = Grid2dSolutionItem.cost;

        return msgSolutionItem;
    }

    /**
     * @brief Convert a MsgSolutionItem type into a Grid2dSolutionItem.
     *
     * @param message Solution item message to convert
     *
     * @return Grid2dSolutionItem generated from the specified MsgSolutionItem
     */
    static Grid2dSolutionItem msg2Grid2dSolutionItem(const srslib_framework::MsgSolutionItem& message)
    {
        Grid2dSolutionItem Grid2dSolutionItem;

        Grid2dSolutionItem.actionType = static_cast<Grid2dSolutionItem::ActionEnum>(message.action);
        Grid2dSolutionItem.fromPose = PoseMessageFactory::msg2Pose(message.fromPose);
        Grid2dSolutionItem.toPose = PoseMessageFactory::msg2Pose(message.toPose);
        Grid2dSolutionItem.cost = message.cost;

        return Grid2dSolutionItem;
    }

    /**
     * @brief Convert a MsgSolutionItemConstPtr type into a Grid2dSolutionItem.
     *
     * @param message Solution item message to convert
     *
     * @return Grid2dSolutionItem generated from the specified MsgSolutionItem
     */
    static Grid2dSolutionItem msg2Grid2dSolutionItem(srslib_framework::MsgSolutionItem::ConstPtr message)
    {
        return msg2Grid2dSolutionItem(*message);
    }

    /**
     * @brief Convert a MsgSolution type into a Solution of Grid2dSolutionItem.
     *
     * @param message Solution message to convert
     *
     * @return Solution of Grid2dSolutionItem generated from the specified MsgSolution
     */
    static Solution<Grid2dSolutionItem> msg2Solution(const srslib_framework::MsgSolution& message)
    {
        Solution<Grid2dSolutionItem> solution;

        for (auto solutionItem : message.items)
        {
            Grid2dSolutionItem Grid2dSolutionItem = msg2Grid2dSolutionItem(solutionItem);

            solution.push_back(Grid2dSolutionItem);
        }

        return solution;
    }

    /**
     * @brief Convert a MsgSolutionConstPtr type into a Solution of Grid2dSolutionItem.
     *
     * @param message Solution message to convert
     *
     * @return Solution of Grid2dSolutionItem generated from the specified MsgSolutionConstPtr
     */
    static Solution<Grid2dSolutionItem> msg2Solution(srslib_framework::MsgSolution::ConstPtr message)
    {
        return msg2Solution(*message);
    }

    /**
     * @brief Convert a Solution of Grid2dSolutionItem into a MsgSolution.
     *
     * @param solution Solution to convert
     * @param timestamp ROS time stamp for the message
     *
     * @return MsgSolution generated from the specified Solution of Grid2dSolutionItem
     */
    static srslib_framework::MsgSolution gridSolution2Msg(const Solution<Grid2dSolutionItem>& solution,
        const ros::Time timestamp = ros::Time::now())
    {
        srslib_framework::MsgSolution msgSolution;
        msgSolution.header.stamp = timestamp;

        vector<srslib_framework::MsgSolutionItem> items;

        for (auto solutionItem : solution)
        {
            items.push_back(Grid2dSolutionItem2Msg(solutionItem));
        }
        msgSolution.items = items;

        return msgSolution;
    }

    /**
     * @brief Convert a Solution of Grid2dSolutionItem into a nav_msgs::Path.
     *
     * @param solution Solution to convert
     * @param timestamp ROS time stamp for the message
     *
     * @return nav_msgs::Path generated from the specified Solution of Grid2dSolutionItem
     */
    static nav_msgs::Path gridSolution2PathMsg(const Solution<Grid2dSolutionItem>& solution,
        const ros::Time timestamp = ros::Time::now())
    {
        nav_msgs::Path messagePath;
        messagePath.header.frame_id = "map";
        messagePath.header.stamp = timestamp;

        vector<geometry_msgs::PoseStamped> planPoses;

        for (auto solutionItem : solution)
        {
            geometry_msgs::PoseStamped poseStamped;
            tf::Quaternion quaternion = tf::createQuaternionFromYaw(solutionItem.toPose.theta);

            poseStamped.pose.position.x = solutionItem.toPose.x;
            poseStamped.pose.position.y = solutionItem.toPose.y;
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
