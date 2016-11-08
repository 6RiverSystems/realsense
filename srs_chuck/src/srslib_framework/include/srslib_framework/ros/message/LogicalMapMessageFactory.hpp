/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <geometry_msgs/Pose.h>

#include <srslib_framework/LogicalMap.h>
#include <srslib_framework/LogicalMetadata.h>

#include <srslib_framework/localization/map/logical/LogicalMap.hpp>
#include <srslib_framework/localization/map/logical/LogicalMetadata.hpp>
#include <srslib_framework/robotics/Pose.hpp>
#include <srslib_framework/ros/message/PoseMessageFactory.hpp>

namespace srs {

struct LogicalMapMessageFactory
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
        vector<srslib_framework::LogicalCell> logical;
        map2Vector(map, logical);

        srslib_framework::LogicalMap msgLogicalMap;

        msgLogicalMap.metadata = metadata2Msg(map->getMetadata());
        msgLogicalMap.data = logical;

        return msgLogicalMap;
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
        msgLogicalMetaData.heightCells = metadata.heightCells;
        msgLogicalMetaData.heightM = metadata.heightM;
        msgLogicalMetaData.logicalFilename = metadata.logicalFilename;
        msgLogicalMetaData.origin = PoseMessageFactory::pose2Msg(metadata.origin);
        msgLogicalMetaData.resolution = metadata.resolution;
        msgLogicalMetaData.widthCells = metadata.widthCells;
        msgLogicalMetaData.widthM = metadata.widthM;

        return msgLogicalMetaData;
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
        LogicalMetadata metadata = msg2Metadata(message.metadata);
        return vector2Map(metadata, message.data);
    }

    /**
     * @brief Convert a MapMetadata message into a LogicalMetaData.
     *
     * @param message LogicalMetadata to convert
     *
     * @return LogicalMetaData generated from the specified MapMetadata message
     */
    static LogicalMetadata msg2Metadata(const srslib_framework::LogicalMetadata& message)
    {
        LogicalMetadata metadata;

        metadata.loadTime = message.loadTime;
        metadata.heightCells = message.heightCells;
        metadata.heightM = message.heightM;
        metadata.logicalFilename = message.logicalFilename;
        metadata.origin = PoseMessageFactory::msg2Pose(message.origin);
        metadata.resolution = message.resolution;
        metadata.widthCells = message.widthCells;
        metadata.widthM = message.widthM;

        return metadata;
    }

private:
    static void map2Vector(const LogicalMap* map,
        vector<srslib_framework::LogicalCell>& logical);

    static LogicalMap* vector2Map(const LogicalMetadata& metadata,
        const vector<srslib_framework::LogicalCell>& logical);
};

} // namespace srs