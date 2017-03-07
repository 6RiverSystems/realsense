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

#include <srslib_framework/localization/map/BaseMap.hpp>
#include <srslib_framework/localization/map/MapStack.hpp>
#include <srslib_framework/localization/map/MapStackMetadata.hpp>
#include <srslib_framework/localization/map/logical/LogicalMap.hpp>
#include <srslib_framework/localization/map/occupancy/OccupancyMap.hpp>

#include <srslib_framework/ros/message/LogicalMapMessageFactory.hpp>
#include <srslib_framework/ros/message/OccupancyMapMessageFactory.hpp>

namespace srs {

struct MapStackMessageFactory
{
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

        msgMapStack.metadata = metadata2Msg(mapStack->getMetadata());
        msgMapStack.logical = LogicalMapMessageFactory::map2Msg(mapStack->getLogicalMap());
        msgMapStack.occupancy = OccupancyMapMessageFactory::map2Msg(mapStack->getOccupancyMap());

        return msgMapStack;
    }

    /**
     * @brief Convert a MapStackMetadata into a MapStackMetadata message.
     *
     * @param metadata Map stack metadata to convert
     *
     * @return newly generated message
     */
    static srslib_framework::MapStackMetadata metadata2Msg(const MapStackMetadata& metadata)
    {
        srslib_framework::MapStackMetadata msgMapStackMetaData;

        msgMapStackMetaData.loadTime = metadata.loadTime;
        msgMapStackMetaData.mapStackFilename = metadata.mapStackFilename;
        msgMapStackMetaData.mapName = metadata.mapName;
        msgMapStackMetaData.mapVersion = metadata.mapVersion;

        return msgMapStackMetaData;
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
        MapStackMetadata metadata = msg2Metadata(message->metadata);
        LogicalMap* logical = LogicalMapMessageFactory::msg2LogicalMap(message->logical);
        OccupancyMap* occupancy = OccupancyMapMessageFactory::msg2OccupancyMap(message->occupancy);

        return new MapStack(metadata, logical, occupancy);
    }

    /**
     * @brief Convert a MapStackMetadata message into a MapStackMetadata.
     *
     * @param message MapStackMetadata message to convert
     *
     * @return MapStackMetadata generated from the specified MapStackMetadata message
     */
    static MapStackMetadata msg2Metadata(const srslib_framework::MapStackMetadata& message)
    {
        MapStackMetadata metadata;

        metadata.loadTime = message.loadTime;
        metadata.mapStackFilename = message.mapStackFilename;
        metadata.mapName = message.mapName;
        metadata.mapVersion = message.mapVersion;

        return metadata;
    }
};

} // namespace srs
