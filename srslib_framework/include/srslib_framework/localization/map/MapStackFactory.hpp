/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>

#include <yaml-cpp/yaml.h>

#include <srslib_framework/localization/map/MapStack.hpp>
#include <srslib_framework/localization/map/logical/LogicalMap.hpp>
#include <srslib_framework/localization/map/occupancy/OccupancyMap.hpp>

namespace srs {

struct MapStackFactory
{
    static const std::string LANGUAGE_VERSION;

    static MapStack* fromJsonFile(string jsonFilename, double loadTime = 0);

private:
    static const std::string TAG_LANGUAGE_VERSION;

    static const std::string TAG_METADATA_SECTION;
    static const std::string TAG_METADATA_MAP_NAME;
    static const std::string TAG_METADATA_MAP_VERSION;

    static const std::string TAG_LOGICAL_SECTION;
    static const std::string TAG_LOGICAL_MAP;

    static const std::string TAG_OCCUPANCY_SECTION;
    static const std::string TAG_OCCUPANCY_MAP;
    static const std::string TAG_OCCUPANCY_RESOLUTION;
    static const std::string TAG_OCCUPANCY_FREE_THRESHOLD;
    static const std::string TAG_OCCUPANCY_OCCUPIED_THRESHOLD;
    static const std::string TAG_OCCUPANCY_NEGATE;
    static const std::string TAG_OCCUPANCY_ORIGIN;

    struct Context {
        std::string jsonFilename;

        std::string localDirectory;

        MapStackMetadata metadata;
    };

    static LogicalMap* analyzeLogicalNode(Context& context, YAML::Node& mapStackDocument);
    static OccupancyMap* analyzeOccupancyNode(Context& context, YAML::Node& mapStackDocument);

    static void checkLanguageVersion(Context& context, YAML::Node& mapStackDocument);

    static void readMetadata(Context& context, YAML::Node& mapStackDocument);
};

} // namespace srs
