/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
using namespace std;

#include <yaml-cpp/yaml.h>

#include <srslib_framework/localization/map/logical/LogicalMap.hpp>

namespace srs {

struct LogicalMapFactory
{
    static LogicalMap* fromJsonFile(string jsonFilename);
    static LogicalMap* fromString(string jsonFilename);

private:
    static void addRectangleCost(LogicalMap* logicalMap,
        Pose<> origin, double width, double height,
        unsigned int cost);
    static void addStaticObstacle(LogicalMap* logicalMap,
        Pose<> origin, double widthM, double heightM,
        double sizeEnvelope = 0.0, unsigned int costEnvelope = 0);

    static bool findId(YAML::Node node, string id, YAML::Node& result);

    static void nonTerminalBoundary(YAML::Node node, LogicalMap* logicalMap);
    static LogicalMap* nonTerminalMap(YAML::Node node);

    static LogicalMap* synthesizeMap(LogicalMetadata& metadata, YAML::Node documentNode);
};

} // namespace srs
