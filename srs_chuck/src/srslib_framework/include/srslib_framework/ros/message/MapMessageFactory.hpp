/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/MsgMap.h>

#include <srslib_framework/localization/map/Map.hpp>

namespace srs {

struct MapMessageFactory
{
    /**
     * @brief Convert a MapMetadata into a MapMetaData message.
     *
     * @param metadata Map metadata to convert
     * @param timestamp ROS time stamp for the message
     *
     * @return newly generated message
     */
    static nav_msgs::MapMetaData mapMetadata2Msg(MapMetadata metadata,
        ros::Time timestamp = ros::Time::now())
    {
        nav_msgs::MapMetaData msgMapMetaData;

        msgMapMetaData.map_load_time = timestamp;
        msgMapMetaData.resolution = metadata.resolution;
        msgMapMetaData.width = metadata.widthCells;
        msgMapMetaData.height = metadata.heightCells;

        geometry_msgs::Pose origin;
        tf::Quaternion orientation = metadata.orientation;

        origin.position.x = metadata.origin.x();
        origin.position.y = metadata.origin.y();
        origin.orientation.x = orientation.x();
        origin.orientation.y = orientation.y();
        origin.orientation.z = orientation.z();
        origin.orientation.w = orientation.w();

        msgMapMetaData.origin = origin;

        return msgMapMetaData;
    }

    /**
     * @brief Convert a MsgMapConstPtr type into a Map.
     *
     * @param message MsgMap to convert
     *
     * @return Map generated from the specified MsgMap
     */
    static Map* msg2Map(srslib_framework::MsgMapConstPtr message)
    {
        Map* map = new Map(message->info.width, message->info.height, message->info.resolution);
        map->setGrid(message->costs, message->notes);

        return map;
    }
};

} // namespace srs
