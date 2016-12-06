/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
using namespace std;

#include <yaml-cpp/yaml.h>

#include <srslib_framework/localization/map/MapStack.hpp>
#include <srslib_framework/localization/map/logical/LogicalMap.hpp>
#include <srslib_framework/localization/map/occupancy/OccupancyMap.hpp>

namespace srs {

struct MapStackFactory
{
    static MapStack* fromJsonFile(string jsonFilename, double loadingTime = 0);

private:
    static const string TAG_LOGICAL;
    static const string TAG_LOGICAL_MAP;

    static const string TAG_OCCUPANCY;
    static const string TAG_OCCUPANCY_RESOLUTION;
    static const string TAG_OCCUPANCY_FREE_THRESHOLD;
    static const string TAG_OCCUPANCY_OCCUPIED_THRESHOLD;
    static const string TAG_OCCUPANCY_NEGATE;
    static const string TAG_OCCUPANCY_ORIGIN;
    static const string TAG_OCCUPANCY_IMAGE;

    static LogicalMap* analizeLogicalNode(string localDirectory, string jsonFilename,
        double loadTime, YAML::Node& mapStackDocument);
    static OccupancyMap* analizeOccupancyNode(string localDirectory, string jsonFilename,
        double loadTime, YAML::Node& mapStackDocument);
};

} // namespace srs
