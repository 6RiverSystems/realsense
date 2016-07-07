/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef SOLUTIONMESSAGEFACTORY_HPP_
#define SOLUTIONMESSAGEFACTORY_HPP_

#include <srslib_framework/MsgPose.h>
#include <srslib_framework/MsgSolutionItem.h>

#include <srslib_framework/planning/pathplanning/grid/GridSolutionItem.hpp>
#include <srslib_framework/ros/message/PoseMessageFactory.hpp>

namespace srs {

struct SolutionMessageFactory
{
    /**
     * @brief Convert a GridSolutionItem type into a MsgSolutionItem.
     *
     * @param gridSolutionItem Solution item to convert
     *
     * @return MsgSolutionItem generated from the specified GridSolutionItem
     */
    static srslib_framework::MsgSolutionItem gridSolutionItem2Msg(GridSolutionItem gridSolutionItem)
    {
        srslib_framework::MsgSolutionItem msgSolutionItem;

        msgSolutionItem.action = gridSolutionItem.actionType;
        msgSolutionItem.fromPose = PoseMessageFactory::pose2Msg(gridSolutionItem.fromPose);
        msgSolutionItem.toPose = PoseMessageFactory::pose2Msg(gridSolutionItem.toPose);
        msgSolutionItem.cost = gridSolutionItem.cost;

        return msgSolutionItem;
    }

    /**
     * @brief Convert a MsgSolutionItem type into a GridSolutionItem.
     *
     * @param message Solution item message to convert
     *
     * @return GridSolutionItem generated from the specified MsgSolutionItem
     */
    static GridSolutionItem msg2GridSolutionItem(srslib_framework::MsgSolutionItem message)
    {
        GridSolutionItem gridSolutionItem;

        gridSolutionItem.actionType = static_cast<GridSolutionItem::ActionEnum>(message.action);
        gridSolutionItem.fromPose = PoseMessageFactory::msg2Pose(message.fromPose);
        gridSolutionItem.toPose = PoseMessageFactory::msg2Pose(message.toPose);
        gridSolutionItem.cost = message.cost;

        return gridSolutionItem;
    }

    /**
     * @brief Convert a MsgSolutionItemConstPtr type into a GridSolutionItem.
     *
     * @param message Solution item message to convert
     *
     * @return GridSolutionItem generated from the specified MsgSolutionItem
     */
    static GridSolutionItem msg2GridSolutionItem(srslib_framework::MsgSolutionItemConstPtr message)
    {
        return SolutionMessageFactory::msg2GridSolutionItem(*message);
    }

    /**
     * @brief Convert a MsgSolution type into a Solution of GridSolutionItem.
     *
     * @param message Solution message to convert
     *
     * @return Solution of GridSolutionItem generated from the specified MsgSolution
     */
    static Solution<GridSolutionItem> msg2Solution(srslib_framework::MsgSolutionConstPtr message)
    {
        Solution<GridSolutionItem> solution;

        for (auto solutionItem : message->items)
        {
            GridSolutionItem gridSolutionItem = SolutionMessageFactory::msg2GridSolutionItem(
                solutionItem);

            solution.push_back(gridSolutionItem);
        }

        return solution;
    }

    /**
     * @brief Convert a Solution of GridSolutionItem into a MsgSolution.
     *
     * @param solution Solution to convert
     * @param timestamp ROS time stamp for the message
     *
     * @return MsgSolution generated from the specified Solution of GridSolutionItem
     */
    static srslib_framework::MsgSolution gridSolution2Msg(Solution<GridSolutionItem>& solution,
        ros::Time timestamp = ros::Time::now())
    {
        srslib_framework::MsgSolution msgSolution;
        msgSolution.header.stamp = timestamp;

        vector<srslib_framework::MsgSolutionItem> items;

        for (auto solutionItem : solution)
        {
            items.push_back(SolutionMessageFactory::gridSolutionItem2Msg(solutionItem));
        }
        msgSolution.items = items;

        return msgSolution;
    }

    /**
     * @brief Convert a Solution of GridSolutionItem into a nav_msgs::Path.
     *
     * @param solution Solution to convert
     * @param timestamp ROS time stamp for the message
     *
     * @return nav_msgs::Path generated from the specified Solution of GridSolutionItem
     */
    static nav_msgs::Path gridSolution2PathMsg(Solution<GridSolutionItem>& solution,
        ros::Time timestamp = ros::Time::now())
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

#endif // SOLUTIONMESSAGEFACTORY_HPP_
