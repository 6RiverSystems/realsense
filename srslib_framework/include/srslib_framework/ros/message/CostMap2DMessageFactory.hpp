/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <costmap_2d/costmap_2d_ros.h>

#include <geometry_msgs/Pose.h>

#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>

#include <srslib_framework/localization/map/occupancy/OccupancyMap.hpp>
#include <srslib_framework/localization/map/occupancy/OccupancyMetadata.hpp>
#include <srslib_framework/robotics/Pose.hpp>
#include <srslib_framework/ros/message/PoseMessageFactory.hpp>

namespace srs {

struct CostMap2DMessageFactory
{
    /**
     * @brief Convert a Costmap2D into a ROS Map Metadata message.
     *
     * @param map Cost map to convert
     * @param loadTime Loading time
     *
     * @return newly generated message
     */
    static nav_msgs::MapMetaData metadata2RosMsg(const costmap_2d::Costmap2D* map,
        const ros::Time loadTime)
    {
        nav_msgs::MapMetaData msgRosMapMetaData;

        msgRosMapMetaData.map_load_time = loadTime;
        msgRosMapMetaData.resolution = map->getResolution();
        msgRosMapMetaData.width = map->getSizeInCellsX();
        msgRosMapMetaData.height = map->getSizeInCellsY();
        msgRosMapMetaData.origin = PoseMessageFactory::pose2RosPose(
            Pose<>(map->getOriginX(), map->getOriginY(), 0));

        return msgRosMapMetaData;
    }

    /**
     * @brief Convert a Cost Map 2D type into a ROS OccupancyGrid message.
     *
     * @param map Cost map to convert
     * @param frameId Frame id of the cost map
     * @param loadTime Loading time
     *
     * @return Ros OccupancyGrid message generated from the specified cost map
     */
    static nav_msgs::OccupancyGrid costMap2RosMsg(const costmap_2d::Costmap2D* map, string frameId,
        const ros::Time loadTime)
    {
        vector<int8_t> occupancy;
        MapAdapter::costMap2D2Vector(map, occupancy);

        nav_msgs::OccupancyGrid msgOccupancyMap;

        msgOccupancyMap.info = metadata2RosMsg(map, loadTime);
        msgOccupancyMap.data = occupancy;
        msgOccupancyMap.header.frame_id = frameId;

        return msgOccupancyMap;
    }
};

} // namespace srs
