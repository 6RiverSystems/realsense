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
#include <srslib_framework/localization/map/logical/LogicalMetadata.hpp>

namespace srs {

struct LogicalMapFactory
{
    LogicalMapFactory() :
        map_(nullptr),
        metadata_()
    {}

    LogicalMap* fromGrid2d(Grid2d* grid, double resolution);
    LogicalMap* fromJsonFile(string jsonFilename, double loadTime = 0);
    LogicalMap* fromString(string jsonFilename, double loadTime = 0);

private:
    static const string KEYWORD_BOUNDARY;

    static const string KEYWORD_COORDINATES;
    static const string KEYWORD_COST_AREA;

    static const string KEYWORD_FEATURE_COLLECTION;
    static const string KEYWORD_FEATURES;

    static const string KEYWORD_GEOMETRY;

    static const string KEYWORD_ID;

    static const string KEYWORD_MAP;
    static const string KEYWORD_MAX;

    static const string KEYWORD_OBSTACLE;

    static const string KEYWORD_PROPERTIES;
    static const string KEYWORD_PROPERTY_COST_AREA_COST;
    static const string KEYWORD_PROPERTY_BOUNDARY_ENVELOPE_COST;
    static const string KEYWORD_PROPERTY_BOUNDARY_ENVELOPE_SIZE;
    static const string KEYWORD_PROPERTY_MAP_HEIGHT;
    static const string KEYWORD_PROPERTY_MAP_ORIGIN;
    static const string KEYWORD_PROPERTY_MAP_RESOLUTION;
    static const string KEYWORD_PROPERTY_MAP_WIDTH;
    static const string KEYWORD_PROPERTY_OBSTACLE_ENVELOPE_COST;
    static const string KEYWORD_PROPERTY_OBSTACLE_ENVELOPE_SIZE;
    static const string KEYWORD_PROPERTY_WEIGHT_AREA_EAST;
    static const string KEYWORD_PROPERTY_WEIGHT_AREA_NORTH;
    static const string KEYWORD_PROPERTY_WEIGHT_AREA_SOUTH;
    static const string KEYWORD_PROPERTY_WEIGHT_AREA_WEST;

    static const string KEYWORD_TYPE;
    static const string KEYWORD_TYPE_POINT;
    static const string KEYWORD_TYPE_MULTIPOINT;

    static const string KEYWORD_WEIGHT_AREA;

    void addRectangleCost(Pose<> origin, double widthM, double heightM,
        Grid2d::BaseType cost);
    void addStaticObstacle(Pose<> origin, double widthM, double heightM,
        double sizeEnvelopeM = 0.0, Grid2d::BaseType costEnvelope = 0);
    void addWeight(Pose<> from, Pose<> to,
        Grid2d::BaseType north,
        Grid2d::BaseType east,
        Grid2d::BaseType south,
        Grid2d::BaseType west);

    bool findIdInCollection(YAML::Node node, string id, YAML::Node& result);
    bool findCollection(YAML::Node node, YAML::Node& result);

    void nonTerminalEntities(YAML::Node node);
    void nonTerminalEntry(YAML::Node node);
    vector<Pose<>> nonTerminalGeometry(YAML::Node node, int minNumber, int maxNumber);
    void nonTerminalMap(YAML::Node node);

    void nonTerminalStatementBoundary(YAML::Node node);
    void nonTerminalStatementCostArea(YAML::Node node);
    void nonTerminalStatementObstacle(YAML::Node node);
    void nonTerminalStatementWeightArea(YAML::Node node);

    Grid2d::BaseType nonTerminalTypeCost(YAML::Node node, bool required);
    double nonTerminalTypeDouble(YAML::Node node, bool required);
    Pose<> nonTerminalTypePoint(YAML::Node node, bool required);

    LogicalMap* map_;
    LogicalMetadata metadata_;
};

} // namespace srs
