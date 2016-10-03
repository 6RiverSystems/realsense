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
#include <srslib_framework/robotics/Pose.hpp>
#include <srslib_framework/ros/message/PoseMessageFactory.hpp>

namespace srs {

struct MapMessageFactory
{
    /**
     * @brief Convert a LogicalMetadata into a LogicalMetaData message.
     *
     * @param metadata Logical metadata to convert
     *
     * @return newly generated message
     */
    static srslib_framework::LogicalMetadata metadata2Msg(LogicalMetadata metadata)
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
    static srslib_framework::OccupancyMetadata metadata2Msg(OccupancyMetadata metadata)
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
    static nav_msgs::MapMetaData metadata2RosMsg(OccupancyMetadata metadata)
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
     * @brief Convert a MapMetadata message into a LogicalMetaData.
     *
     * @param message MapMetadata to convert
     *
     * @return LogicalMetaData generated from the specified MapMetadata message
     */
    static LogicalMetadata msg2LogicalMetadata(srslib_framework::MapStackMetadata::ConstPtr message)
    {
        LogicalMetadata metadata;

        metadata.loadTime = message->logical.loadTime;
        metadata.logicalFilename = message->logical.logicalFilename;

        return metadata;
    }

    /**
     * @brief Convert a MapMetadata message into a OccupancyMetaData.
     *
     * @param message MapMetadata to convert
     *
     * @return OccupancyMetaData generated from the specified MapMetadata message
     */
    static OccupancyMetadata msg2OccupancyMetadata(srslib_framework::MapStackMetadata::ConstPtr message)
    {
        OccupancyMetadata metadata;

        metadata.loadTime = message->occupancy.loadTime;
        metadata.heightCells = message->occupancy.heightCells;
        metadata.heightM = message->occupancy.heightM;
        metadata.occupancyFilename = message->occupancy.occupancyFilename;
        metadata.negate = message->occupancy.negate;
        metadata.origin = PoseMessageFactory::msg2Pose(message->occupancy.origin);
        metadata.resolution = message->occupancy.resolution;
        metadata.thresholdFree = message->occupancy.thresholdFree;
        metadata.thresholdOccupied = message->occupancy.thresholdOccupied;
        metadata.widthCells = message->occupancy.widthCells;
        metadata.widthM = message->occupancy.widthM;

        return metadata;
    }

    /**
     * @brief Convert the metadata of a Map Stack into a MapMetaData message.
     *
     * @param mapStack Map stack to use for metadata generation
     *
     * @return newly generated message
     */
    static srslib_framework::MapStackMetadata metadata2Msg(MapStack* mapStack)
    {
        srslib_framework::MapStackMetadata msgMapMetaData;

        msgMapMetaData.logical = metadata2Msg(mapStack->getLogicalMap()->getMetadata());
        msgMapMetaData.occupancy = metadata2Msg(mapStack->getOccupancyMap()->getMetadata());

        return msgMapMetaData;
    }

//    /**
//     * @brief Convert a MsgMapConstPtr type into a Map.
//     *
//     * @param message MsgMap to convert
//     *
//     * @return Map generated from the specified MsgMap
//     */
//    static Map* msg2Map(srslib_framework::MapConstPtr message)
//    {
//        Map* map = new Map(message->info.width, message->info.height, message->info.resolution);
//        map->setGrid(message->costs, message->notes);
//
//        return map;
//    }
};

} // namespace srs
