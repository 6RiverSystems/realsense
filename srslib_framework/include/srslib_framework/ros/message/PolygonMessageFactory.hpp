/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point.h>

#include <srslib_framework/Pose.h>

#include <srslib_framework/math/TimeMath.hpp>
#include <srslib_framework/robotics/Pose.hpp>

namespace srs {

struct PolygonMessageFactory
{
    /**
     * @brief Convert a vector of Pose type into a PolygonStamped.
     *
     * @param poses vector of Pose to convert
     * @param timestamp ROS time stamp for the message
     *
     * @return PolygonStamped generated from the specified Poses
     */
    static geometry_msgs::PolygonStamped poses2PolygonStamped(const std::vector<Pose<>>& poses,
        ros::Time timestamp = ros::Time::now())
    {
        geometry_msgs::PolygonStamped message;
        message.header.stamp = timestamp;

        vector<geometry_msgs::Point32> polygon;
        for (auto pose : poses)
        {
            geometry_msgs::Point32 pt;
            pt.x = pose.x;
            pt.y = pose.y;
            pt.z = 0.0;

            polygon.push_back(pt);
        }
        message.polygon.points = polygon;

        return message;
    }

    /**
     * @brief Convert a vector of points type into a vector of Pose.
     *
     * @param points vector of points to convert
     *
     * @return vector of Poses generated from the specified points
     */
    static std::vector<Pose<>> points2Poses(const std::vector<geometry_msgs::Point>& points)
    {
        std::vector<Pose<>> poses;
        for (auto pt : points)
        {
            poses.push_back(Pose<>(pt.x, pt.y, 0.0));
        }

        return poses;
    }

    /**
     * @brief Convert a PolygonStamped type into a vectof of Pose.
     *
     * @param message PolygonStamped to convert
     *
     * @return vector of Poses generated from the specified PolygonStamped
     */
    static std::vector<Pose<>> polygonStamped2Poses(const geometry_msgs::PolygonStamped& message)
    {
        std::vector<Pose<>> poses;
        for (auto pt : message.polygon.points)
        {
            poses.push_back(Pose<>(pt.x, pt.y, 0.0));
        }

        return poses;
    }

    /**
     * @brief Convert a PolygonStampedConstPtr type into a Pose.
     *
     * @param message PolygonStampedConstPtr to convert
     *
     * @return vector of Pose generated from the specified PolygonStampedConstPtr
     */
    static std::vector<Pose<>> polygonStamped2Poses(geometry_msgs::PolygonStamped::ConstPtr message)
    {
        return polygonStamped2Poses(*message);
    }
};

} // namespace srs
