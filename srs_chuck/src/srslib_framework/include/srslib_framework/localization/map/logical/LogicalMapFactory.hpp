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
        Pose<> origin, double widthMm, double heightMm,
        unsigned int cost);
    static void addStaticObstacle(LogicalMap* logicalMap,
        Pose<> origin, double widthMm, double heightMm,
        double sizeEnvelope = 0.0, unsigned int costEnvelope = 0);

    static bool findIdInCollection(YAML::Node node, string id, YAML::Node& result);
    static bool findCollection(YAML::Node node, YAML::Node& result);

    static void nonTerminalBoundary(YAML::Node node, LogicalMap* map);
    static LogicalMap* nonTerminalEntry(LogicalMetadata& metadata, YAML::Node node);
    static LogicalMap* nonTerminalMap(YAML::Node node);
    static void nonTerminalObstacle(YAML::Node node, LogicalMap* map);
    static void nonTerminalObstacles(YAML::Node node, LogicalMap* map);
};

} // namespace srs
