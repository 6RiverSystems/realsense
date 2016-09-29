/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <geometry_msgs/Pose.h>

#include <srslib_framework/Map.h>
#include <srslib_framework/MapMetadata.h>

#include <srslib_framework/localization/map/Map.hpp>
#include <srslib_framework/localization/map/MapMetadata.hpp>
#include <srslib_framework/robotics/Pose.hpp>
#include <srslib_framework/ros/message/PoseMessageFactory.hpp>

namespace srs {

struct MapMessageFactory
{
    /**
     * @brief Convert a MapMetadata into a MapMetaData message.
     *
     * @param metadata Map metadata to convert
     *
     * @return newly generated message
     */
    static srslib_framework::MapMetadata mapMetadata2Msg(MapMetadata metadata)
    {
        srslib_framework::MapMetadata msgMapMetaData;

        msgMapMetaData.loadTime = metadata.loadTime;
        msgMapMetaData.heightCells = metadata.heightCells;
        msgMapMetaData.heightM = metadata.heightM;
        msgMapMetaData.mapDocumentFilename = metadata.mapDocumentFilename;
        msgMapMetaData.mapImageFilename = metadata.mapImageFilename;
        msgMapMetaData.negate = metadata.negate;
        msgMapMetaData.origin = PoseMessageFactory::pose2Msg(metadata.origin);
        msgMapMetaData.resolution = metadata.resolution;
        msgMapMetaData.thresholdFree = metadata.thresholdFree;
        msgMapMetaData.thresholdOccupied = metadata.thresholdOccupied;
        msgMapMetaData.widthCells = metadata.widthCells;
        msgMapMetaData.widthM = metadata.widthM;

        return msgMapMetaData;
    }

    /**
     * @brief Convert a MapMetadata into a ROS MapMetaData message.
     *
     * @param metadata Map metadata to convert
     *
     * @return newly generated message
     */
    static nav_msgs::MapMetaData mapMetadata2RosMsg(MapMetadata metadata)
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
     * @brief Convert a MsgMapConstPtr type into a Map.
     *
     * @param message MsgMap to convert
     *
     * @return Map generated from the specified MsgMap
     */
    static Map* msg2Map(srslib_framework::MapConstPtr message)
    {
        Map* map = new Map(message->info.width, message->info.height, message->info.resolution);
        map->setGrid(message->costs, message->notes);

        return map;
    }

    /**
     * @brief Convert a MapMetadata message into a MapMetaData.
     *
     * @param message MapMetadata to convert
     *
     * @return MapMetadata generated from the specified MapMetadata message
     */
    static MapMetadata msg2MapMetadata(srslib_framework::MapMetadata::ConstPtr message)
    {
        MapMetadata metadata;

        metadata.loadTime = message->loadTime;
        metadata.heightCells = message->heightCells;
        metadata.heightM = message->heightM;
        metadata.mapDocumentFilename = message->mapDocumentFilename;
        metadata.mapImageFilename = message->mapImageFilename;
        metadata.negate = message->negate;
        metadata.origin = PoseMessageFactory::msg2Pose(message->origin);
        metadata.resolution = message->resolution;
        metadata.thresholdFree = message->thresholdFree;
        metadata.thresholdOccupied = message->thresholdOccupied;
        metadata.widthCells = message->widthCells;
        metadata.widthM = message->widthM;

        return metadata;
    }
};

} // namespace srs
