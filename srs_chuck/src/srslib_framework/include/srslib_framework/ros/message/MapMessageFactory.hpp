/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <geometry_msgs/Pose.h>

#include <srslib_framework/MapStack.h>
#include <srslib_framework/MapStackMetadata.h>
#include <srslib_framework/LogicalMetadata.h>
#include <srslib_framework/OccupancyMetadata.h>

#include <srslib_framework/localization/map/MapStack.hpp>
#include <srslib_framework/localization/map/logical/LogicalMetadata.hpp>
#include <srslib_framework/localization/map/occupancy/OccupancyMetadata.hpp>
#include <srslib_framework/localization/map/occupancy/OccupancyMapUtils.hpp>
#include <srslib_framework/robotics/Pose.hpp>
#include <srslib_framework/ros/message/PoseMessageFactory.hpp>

namespace srs {

struct MapMessageFactory
{
    /**
     * @brief Convert a Logical Map type into a LogicalMap message.
     *
     * @param map Logical map to convert
     *
     * @return LogicalMap message generated from the specified logical map
     */
    static srslib_framework::LogicalMap map2Msg(const LogicalMap* map)
    {
        srslib_framework::LogicalMap msgLogicalMap;

        msgLogicalMap.metadata = MapMessageFactory::metadata2Msg(map->getMetadata());

        return msgLogicalMap;
    }

    /**
     * @brief Convert a Logical Map type into a LogicalMap message.
     *
     * @param map Logical map to convert
     *
     * @return LogicalMap message generated from the specified logical map
     */
    static srslib_framework::OccupancyMap map2Msg(const OccupancyMap* map)
    {
        vector<int8_t> occupancy;
        OccupancyMapUtils::map2Occupancy(map, occupancy);

        srslib_framework::OccupancyMap msgOccupancyMap;

        msgOccupancyMap.metadata = MapMessageFactory::metadata2Msg(map->getMetadata());
        msgOccupancyMap.data = occupancy;

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

        msgMapStack.logical = MapMessageFactory::map2Msg(mapStack->getLogicalMap());
        msgMapStack.occupancy = MapMessageFactory::map2Msg(mapStack->getOccupancyMap());

        return msgMapStack;
    }

    /**
     * @brief Convert a LogicalMetadata into a LogicalMetaData message.
     *
     * @param metadata Logical metadata to convert
     *
     * @return newly generated message
     */
    static srslib_framework::LogicalMetadata metadata2Msg(const LogicalMetadata& metadata)
    {
        srslib_framework::LogicalMetadata msgLogicalMetaData;

        msgLogicalMetaData.loadTime = metadata.loadTime;
        msgLogicalMetaData.logicalFilename = metadata.logicalFilename;

        return msgLogicalMetaData;
    }

    /**
     * @brief Convert a OccupancyMetadata into a OccupancyMetadata message.
     *
     * @param metadata Occupancy metadata to convert
     *
     * @return newly generated message
     */
    static srslib_framework::OccupancyMetadata metadata2Msg(const OccupancyMetadata& metadata)
    {
        srslib_framework::OccupancyMetadata msgOccupancyMetaData;

        msgOccupancyMetaData.loadTime = metadata.loadTime;
        msgOccupancyMetaData.heightCells = metadata.heightCells;
        msgOccupancyMetaData.heightM = metadata.heightM;
        msgOccupancyMetaData.occupancyFilename = metadata.occupancyFilename;
        msgOccupancyMetaData.negate = metadata.negate;
        msgOccupancyMetaData.origin = PoseMessageFactory::pose2Msg(metadata.origin);
        msgOccupancyMetaData.resolution = metadata.resolution;
        msgOccupancyMetaData.thresholdFree = metadata.thresholdFree;
        msgOccupancyMetaData.thresholdOccupied = metadata.thresholdOccupied;
        msgOccupancyMetaData.widthCells = metadata.widthCells;
        msgOccupancyMetaData.widthM = metadata.widthM;

        return msgOccupancyMetaData;
    }

    /**
     * @brief Convert a OccupancyMetadata into a ROS Map Metadata message.
     *
     * @param metadata Occupancy metadata to convert
     *
     * @return newly generated message
     */
    static nav_msgs::MapMetaData metadata2RosMsg(const OccupancyMetadata& metadata)
    {
        nav_msgs::MapMetaData msgRosMapMetaData;

        msgRosMapMetaData.map_load_time = ros::Time(metadata.loadTime);
        msgRosMapMetaData.resolution = metadata.resolution;
        msgRosMapMetaData.width = metadata.widthCells;
        msgRosMapMetaData.height = metadata.heightCells;
        msgRosMapMetaData.origin = PoseMessageFactory::pose2RosPose(metadata.origin);

        return msgRosMapMetaData;
    }

    /**
     * @brief Convert a LogicalMap message into a LogicalMap.
     *
     * @param message LogicalMap to convert
     *
     * @return LogicalMap generated from the specified LogicalMap message
     */
    static LogicalMap* msg2LogicalMap(const srslib_framework::LogicalMap& message)
    {
        LogicalMap* logical = new LogicalMap(message.metadata.widthCells,
            message.metadata.heightCells, message.metadata.resolution);

        return logical;
    }

    /**
     * @brief Convert a OccupancyMap message into a OccupancyMap.
     *
     * @param message OccupancyMap to convert
     *
     * @return OccupancyMap generated from the specified OccupancyMap message
     */
    static OccupancyMap* msg2OccupancyMap(const srslib_framework::OccupancyMap& message)
    {
        OccupancyMetadata metadata = MapMessageFactory::msg2OccupancyMetadata(message.metadata);
        OccupancyMap* occupancyMap = OccupancyMapUtils::occupancy2Map(metadata, message.data);

        return occupancyMap;
    }

    /**
     * @brief Convert a MapMetadata message into a LogicalMetaData.
     *
     * @param message LogicalMetadata to convert
     *
     * @return LogicalMetaData generated from the specified MapMetadata message
     */
    static LogicalMetadata msg2LogicalMetadata(const srslib_framework::LogicalMetadata& message)
    {
        LogicalMetadata metadata;

        metadata.loadTime = message.loadTime;
        metadata.logicalFilename = message.logicalFilename;

        return metadata;
    }

    /**
     * @brief Convert a MapMetadata message into a OccupancyMetaData.
     *
     * @param message OccupancyMetadata to convert
     *
     * @return OccupancyMetaData generated from the specified MapMetadata message
     */
    static OccupancyMetadata msg2OccupancyMetadata(const srslib_framework::OccupancyMetadata& message)
    {
        OccupancyMetadata metadata;

        metadata.loadTime = message.loadTime;
        metadata.heightCells = message.heightCells;
        metadata.heightM = message.heightM;
        metadata.occupancyFilename = message.occupancyFilename;
        metadata.negate = message.negate;
        metadata.origin = PoseMessageFactory::msg2Pose(message.origin);
        metadata.resolution = message.resolution;
        metadata.thresholdFree = message.thresholdFree;
        metadata.thresholdOccupied = message.thresholdOccupied;
        metadata.widthCells = message.widthCells;
        metadata.widthM = message.widthM;

        return metadata;
    }

    /**
     * @brief Convert the metadata of a Map Stack into a MapMetaData message.
     *
     * @param mapStack Map stack to use for metadata generation
     *
     * @return newly generated message
     */
    static srslib_framework::MapStackMetadata metadata2Msg(const MapStack* mapStack)
    {
        srslib_framework::MapStackMetadata msgMapMetaData;

        msgMapMetaData.logical = metadata2Msg(mapStack->getLogicalMap()->getMetadata());
        msgMapMetaData.occupancy = metadata2Msg(mapStack->getOccupancyMap()->getMetadata());

        return msgMapMetaData;
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
        LogicalMap* logical = MapMessageFactory::msg2LogicalMap(message->logical);
        OccupancyMap* occupancy = MapMessageFactory::msg2OccupancyMap(message->occupancy);

        return new MapStack(logical, occupancy);
    }

    /**
     * @brief Convert a Occupancy Map type into a ROS OccupancyGrid message.
     *
     * @param map Occupancy map to convert
     *
     * @return Ros OccupancyGrid message generated from the specified occupancy map
     */
    static nav_msgs::OccupancyGrid occupancyMap2Msg(const OccupancyMap* map)
    {
        vector<int8_t> occupancy;
        OccupancyMapUtils::map2Occupancy(map, occupancy);

        nav_msgs::OccupancyGrid msgOccupancyMap;

        msgOccupancyMap.info = MapMessageFactory::metadata2RosMsg(map->getMetadata());
        msgOccupancyMap.data = occupancy;
        msgOccupancyMap.header.frame_id = "map";

        return msgOccupancyMap;
    }

};

} // namespace srs
