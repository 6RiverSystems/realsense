/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <geometry_msgs/Pose.h>

#include <srslib_framework/MapStack.h>
#include <srslib_framework/LogicalMap.h>
#include <srslib_framework/LogicalMetadata.h>
#include <srslib_framework/OccupancyMap.h>
#include <srslib_framework/OccupancyMetadata.h>

#include <srslib_framework/localization/map/MapStack.hpp>
#include <srslib_framework/localization/map/BaseMap.hpp>
#include <srslib_framework/localization/map/logical/LogicalMap.hpp>
#include <srslib_framework/localization/map/occupancy/OccupancyMap.hpp>

#include <srslib_framework/ros/message/LogicalMapMessageFactory.hpp>
#include <srslib_framework/ros/message/OccupancyMapMessageFactory.hpp>

namespace srs {

struct MapStackMessageFactory
{
    /**
     * @brief Convert a BaseMap type into a ROS OccupancyGrid message.
     *
     * @param map BaseMap to convert
     * @param frameId Frame id of the occupancy map
     *
     * @return Ros OccupancyGrid message generated from the specified BaseMap
     */
    static nav_msgs::OccupancyGrid baseMap2RosMsg(const BaseMap* map, string frameId)
    {
        vector<int8_t> occupancy;
        MapAdapter::baseMap2Vector(map, occupancy);

        nav_msgs::OccupancyGrid msgOccupancyMap;

        msgOccupancyMap.info = metadata2RosMsg(map);
        msgOccupancyMap.data = occupancy;
        msgOccupancyMap.header.frame_id = frameId;

        return msgOccupancyMap;
    }

    /**
     * @brief Convert a MapStack type into a MapStack message.
     *
     * @param mapStack MapStack to convert
     *
     * @return MapStack message generated from the specified MapStack
     */
    static srslib_framework::MapStack mapStack2Msg(const MapStack* mapStack)
    {
        srslib_framework::MapStack msgMapStack;

        msgMapStack.logical = LogicalMapMessageFactory::map2Msg(mapStack->getLogicalMap());
        msgMapStack.occupancy = OccupancyMapMessageFactory::map2Msg(mapStack->getOccupancyMap());

        return msgMapStack;
    }

    /**
     * @brief Extract from a BaseMap, a ROS Map Metadata message.
     *
     * @param metadata BaseMap to use
     *
     * @return newly generated message
     */
    static nav_msgs::MapMetaData metadata2RosMsg(const BaseMap* map)
    {
        nav_msgs::MapMetaData msgRosMapMetaData;

        msgRosMapMetaData.map_load_time = ros::Time::now();
        msgRosMapMetaData.resolution = map->getResolution();
        msgRosMapMetaData.width = map->getWidthCells();
        msgRosMapMetaData.height = map->getHeightCells();
        msgRosMapMetaData.origin = PoseMessageFactory::pose2RosPose(map->getOrigin());

        return msgRosMapMetaData;
    }

    /**
     * @brief Convert a MapStack message ConstPtr type into a MapStack.
     *
     * @param message MapStack to convert
     *
     * @return MapStack generated from the specified MapStack message
     */
    static MapStack* msg2MapStack(srslib_framework::MapStack::ConstPtr message)
    {
        LogicalMap* logical = LogicalMapMessageFactory::msg2LogicalMap(message->logical);
        OccupancyMap* occupancy = OccupancyMapMessageFactory::msg2OccupancyMap(message->occupancy);

        return new MapStack(logical, occupancy);
    }
};

} // namespace srs
