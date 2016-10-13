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
    static void addWeight(LogicalMap* logicalMap,
        Pose<> from, Pose<> to,
        int north, int east, int south, int west);

    static bool findIdInCollection(YAML::Node node, string id, YAML::Node& result);
    static bool findCollection(YAML::Node node, YAML::Node& result);

    static int nonTerminalCostValue(YAML::Node node, LogicalMap* map);
    static void nonTerminalEntities(YAML::Node node, LogicalMap* map);
    static LogicalMap* nonTerminalEntry(LogicalMetadata& metadata, YAML::Node node);
    static vector<Pose<>> nonTerminalGeometry(YAML::Node node, LogicalMap* map);
    static LogicalMap* nonTerminalMap(YAML::Node node);

    static void nonTerminalStatementBoundary(YAML::Node node, LogicalMap* map);
    static void nonTerminalStatementObstacle(YAML::Node node, LogicalMap* map);
    static void nonTerminalStatementWeight(YAML::Node node, LogicalMap* map);
};

} // namespace srs