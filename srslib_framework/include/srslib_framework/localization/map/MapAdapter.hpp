/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
using namespace std;

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>

#include <srslib_framework/localization/map/occupancy/OccupancyMetadata.hpp>
#include <srslib_framework/localization/map/occupancy/OccupancyMap.hpp>

#include <srslib_framework/localization/map/logical/LogicalMetadata.hpp>
#include <srslib_framework/localization/map/logical/LogicalMap.hpp>

namespace srs {

struct MapAdapter
{
    /**
     * @brief Convert a Cost Map type into a vector of integers.
     *
     * @param costmap Cost Map to convert
     * @param int8Vector Reference to the vector of integers
     */
    static void costMap2D2Vector(const costmap_2d::Costmap2D* costmap, vector<int8_t>& int8Vector);

    /**
     * @brief Convert a Occupancy Map type into a Cost Map.
     *
     * @param occupancy Occupancy Map to convert
     *
     * @return Pointer to the Cost Map
     */
    static costmap_2d::Costmap2D* map2CostMap2D(OccupancyMap* occupancy);

    /**
     * @brief Convert a Logical Map type into a Cost Map.
     *
     * @param logical Logical Map to convert
     *
     * @return Pointer to the Cost Map
     */
    static costmap_2d::Costmap2D* map2CostMap2D(LogicalMap* logical);

    /**
     * @brief Convert a Occupancy Map type into a vector of integers with values
     * specifically expected by the current ROS AMCL implementation.
     *
     * @param occupancy Occupancy Map to convert
     * @param int8Vector Reference to the vector of integers
     */
    static void occupancyMap2AmclVector(const OccupancyMap* occupancy, vector<int8_t>& int8Vector);

    /**
     * @brief Convert a Occupancy Map type into a vector of integers.
     *
     * @param occupancy Occupancy Map to convert
     * @param int8Vector Reference to the vector of integers
     */
    static void occupancyMap2Vector(const OccupancyMap* occupancy, vector<int8_t>& int8Vector);

    /**
     * @brief Convert a vector of integers into a Occupancy Map.
     *
     * @param metadata Occupancy Metadata needed to generate the Occupancy Map
     * @param int8Vector Reference to the vector of integers
     *
     * @return Pointer to the Occupancy Map
     */
    static OccupancyMap* vector2Map(const OccupancyMetadata& metadata,
        const vector<int8_t>& int8Vector);

    /**
     * @brief Extract the weight information of a Logical Map type and store it into a Cost Map.
     *
     * @param logical Logical Map to use
     * @param orientation Specify which weights to extract
     *
     * @return Pointer to the Cost Map
     */
    static costmap_2d::Costmap2D* weights2CostMap2D(LogicalMap* logical, int orientation);
};

} // namespace srs
