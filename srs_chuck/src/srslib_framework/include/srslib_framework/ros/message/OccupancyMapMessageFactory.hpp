/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <geometry_msgs/Pose.h>

#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>
#include <srslib_framework/OccupancyMap.h>
#include <srslib_framework/OccupancyMetadata.h>

#include <srslib_framework/localization/map/MapAdapter.hpp>
#include <srslib_framework/localization/map/occupancy/OccupancyMap.hpp>
#include <srslib_framework/localization/map/occupancy/OccupancyMapFactory.hpp>
#include <srslib_framework/localization/map/occupancy/OccupancyMetadata.hpp>
#include <srslib_framework/robotics/Pose.hpp>
#include <srslib_framework/ros/message/PoseMessageFactory.hpp>

namespace srs {

struct OccupancyMapMessageFactory
{
    /**
     * @brief Convert a Occupancy Map type into a LogicalMap message.
     *
     * @param map Logical map to convert
     *
     * @return LogicalMap message generated from the specified logical map
     */
    static srslib_framework::OccupancyMap map2Msg(const OccupancyMap* map)
    {
        vector<int8_t> int8Vector;
        MapAdapter::occupancyMap2Vector(map, int8Vector);

        srslib_framework::OccupancyMap msgOccupancyMap;

        msgOccupancyMap.metadata = metadata2Msg(map->getMetadata());
        msgOccupancyMap.data = int8Vector;

        return msgOccupancyMap;
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
     * @brief Convert a OccupancyMap message into a OccupancyMap.
     *
     * @param message OccupancyMap to convert
     *
     * @return OccupancyMap generated from the specified OccupancyMap message
     */
    static OccupancyMap* msg2OccupancyMap(const srslib_framework::OccupancyMap& message)
    {
        OccupancyMetadata metadata = msg2Metadata(message.metadata);

        OccupancyMapFactory occupancyMapFactory;
        return occupancyMapFactory.fromMetadata(metadata, message.data);
    }

    /**
     * @brief Convert a OccupancyGrid message into a OccupancyMap.
     *
     * @param message OccupancyGrid to convert
     *
     * @return OccupancyMap generated from the specified OccupancyGrid message
     */
    static OccupancyMap* msg2OccupancyMap(const nav_msgs::OccupancyGrid& message)
    {
        OccupancyMetadata metadata = msg2Metadata(message.info);

        OccupancyMapFactory occupancyMapFactory;
        return occupancyMapFactory.fromMetadata(metadata, message.data);
    }

    /**
     * @brief Convert a MapMetadata message into a OccupancyMetaData.
     *
     * @param message OccupancyMetadata to convert
     *
     * @return OccupancyMetaData generated from the specified MapMetadata message
     */
    static OccupancyMetadata msg2Metadata(const srslib_framework::OccupancyMetadata& message)
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
     * @brief Convert a MapMetadata message into a OccupancyMetaData.
     *
     * @param message OccupancyMetadata to convert
     *
     * @return OccupancyMetaData generated from the specified MapMetadata message
     */
    static OccupancyMetadata msg2Metadata(const nav_msgs::MapMetaData& message)
    {
        OccupancyMetadata metadata;

        metadata.loadTime = message.map_load_time.toSec();
        metadata.resolution = message.resolution;
        metadata.widthCells = message.width;
        metadata.heightCells = message.height;
        metadata.origin = PoseMessageFactory::msg2Pose(message.origin);

        return metadata;
    }

    /**
     * @brief Convert a Occupancy Map type into a ROS OccupancyGrid message with
     * specific values for the current ROS AMCL implementation.
     *
     * @param map Occupancy map to convert
     * @param frameId Frame id of the occupancy map
     *
     * @return Ros OccupancyGrid message generated from the specified occupancy map
     */
    static nav_msgs::OccupancyGrid occupancyMap2RosAmclMsg(const OccupancyMap* map, string frameId)
    {
        vector<int8_t> int8Vector;
        MapAdapter::occupancyMap2AmclVector(map, int8Vector);

        nav_msgs::OccupancyGrid msgOccupancyMap;

        msgOccupancyMap.info = metadata2RosMsg(map->getMetadata());
        msgOccupancyMap.data = int8Vector;
        msgOccupancyMap.header.frame_id = frameId;

        return msgOccupancyMap;
    }

    /**
     * @brief Convert a Occupancy Map type into a ROS OccupancyGrid message.
     *
     * @param map Occupancy map to convert
     * @param frameId Frame id of the occupancy map
     *
     * @return Ros OccupancyGrid message generated from the specified occupancy map
     */
    static nav_msgs::OccupancyGrid occupancyMap2RosMsg(const OccupancyMap* map, string frameId)
    {
        vector<int8_t> int8Vector;
        MapAdapter::occupancyMap2Vector(map, int8Vector);

        nav_msgs::OccupancyGrid msgOccupancyMap;

        msgOccupancyMap.info = metadata2RosMsg(map->getMetadata());
        msgOccupancyMap.data = int8Vector;
        msgOccupancyMap.header.frame_id = frameId;

        return msgOccupancyMap;
    }
};

} // namespace srs
