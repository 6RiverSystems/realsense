/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
using namespace std;

#include <yaml-cpp/yaml.h>
#include <costmap_2d/costmap_2d_ros.h>

#include <srslib_framework/localization/map/MapNote.hpp>
#include <srslib_framework/localization/map/logical/LogicalMap.hpp>
#include <srslib_framework/localization/map/logical/LogicalMetadata.hpp>

namespace srs {

class LogicalMapFactory
{
public:
    LogicalMapFactory() :
        map_(nullptr),
        metadata_()
    {}

    LogicalMap* fromCostMap2D(costmap_2d::Costmap2D* costMap);
    LogicalMap* fromGrid2d(Grid2d* grid, double resolution, Pose<> origin);
    LogicalMap* fromJsonFile(string jsonFilename, double loadTime = 0);
    LogicalMap* fromString(string geoJson, double loadTime = 0);

private:
    static const string KEYWORD_BOUNDARY;
    static const string KEYWORD_COORDINATES;
    static const string KEYWORD_COST_AREA;
    static const string KEYWORD_EDGE;
    static const string KEYWORD_FEATURE_COLLECTION;
    static const string KEYWORD_FEATURES;
    static const string KEYWORD_GEOMETRY;
    static const string KEYWORD_GRAPH;
    static const string KEYWORD_ID;
    static const string KEYWORD_LABELED_AREA;
    static const string KEYWORD_MAP;
    static const string KEYWORD_MAX;
    static const string KEYWORD_NULL;
    static const string KEYWORD_OBJECT;
    static const string KEYWORD_OBSTACLE;
    static const string KEYWORD_PROPERTIES;
    static const string KEYWORD_PROPERTY_COST_AREA_COST;
    static const string KEYWORD_PROPERTY_BOUNDARY_ENVELOPE_COST;
    static const string KEYWORD_PROPERTY_BOUNDARY_ENVELOPE_SIZE;
    static const string KEYWORD_PROPERTY_FEATURE_OBJECT;
    static const string KEYWORD_PROPERTY_LABEL_AREA_LABEL;
    static const string KEYWORD_PROPERTY_LABEL_AREA_BEEP;
    static const string KEYWORD_PROPERTY_LABEL_AREA_NOTES;
    static const string KEYWORD_PROPERTY_MAP_HEIGHT;
    static const string KEYWORD_PROPERTY_MAP_ORIGIN;
    static const string KEYWORD_PROPERTY_MAP_RESOLUTION;
    static const string KEYWORD_PROPERTY_MAP_WIDTH;
    static const string KEYWORD_PROPERTY_OBSTACLE_ENVELOPE_COST;
    static const string KEYWORD_PROPERTY_OBSTACLE_ENVELOPE_SIZE;
    static const string KEYWORD_PROPERTY_WEIGHTED_AREA_EAST;
    static const string KEYWORD_PROPERTY_WEIGHTED_AREA_NORTH;
    static const string KEYWORD_PROPERTY_WEIGHTED_AREA_SOUTH;
    static const string KEYWORD_PROPERTY_WEIGHTED_AREA_WEST;
    static const string KEYWORD_TYPE;
    static const string KEYWORD_TYPE_POINT;
    static const string KEYWORD_TYPE_MULTIPOINT;
    static const string KEYWORD_VERTEX;
    static const string KEYWORD_WEIGHTED_AREA;

    void addCostArea(Pose<> origin, double widthM, double heightM,
        Grid2d::BaseType cost);
    void addLabelArea(Pose<> origin, double widthM, double heightM,
        string label, MapNote note);
    void addObstacleArea(Pose<> origin, double widthM, double heightM,
        double sizeEnvelopeM = 0.0, Grid2d::BaseType costEnvelope = 0);
    void addWeightArea(Pose<> origin, double widthM, double heightM,
        Grid2d::BaseType north,
        Grid2d::BaseType east,
        Grid2d::BaseType south,
        Grid2d::BaseType west);

    void calculateArea(Pose<> origin, double widthM, double heightM,
        unsigned int& x0, unsigned int& y0, unsigned int& widthCells, unsigned int& heightCells);

    bool findCollection(YAML::Node root, YAML::Node& result);
    bool findEntityInCollection(YAML::Node root, string entityType, YAML::Node& result);
    string findEntityObject(YAML::Node root);

    void ntEntities(YAML::Node root);
    void ntEntityBoundary(YAML::Node root);
    void ntEntityCostArea(YAML::Node root);
    void ntEntityEdge(YAML::Node root);
    void ntEntityGraph(YAML::Node root);
    void ntEntityLabelArea(YAML::Node root);
    void ntEntityMap(YAML::Node root);
    void ntEntityObstacle(YAML::Node root);
    void ntEntityVertex(YAML::Node root);
    void ntEntityWeightArea(YAML::Node root);
    void ntEntry(YAML::Node root);
    vector<Pose<>> ntGeometry(YAML::Node root, int minNumber, int maxNumber);
    void ntGeometryNull(YAML::Node root);
    YAML::Node ntProperties(YAML::Node root);

    Grid2d::BaseType ntValueCost(YAML::Node root, bool required);
    double ntValueDouble(YAML::Node root, bool required);
    MapNote ntValueMapNote(YAML::Node root, bool required);
    vector<Pose<>> ntValueMultiPoint(YAML::Node root, bool required);
    Pose<> ntValuePoint(YAML::Node root, bool required);
    string ntValueString(YAML::Node root, bool required);

    LogicalMap* map_;
    LogicalMetadata metadata_;
};

} // namespace srs
