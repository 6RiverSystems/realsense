/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <vector>
#include <algorithm>

#include <geometry_msgs/Point.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>

#include <srslib_framework/localization/map/logical/LogicalMap.hpp>

#include <srsnode_navigation/global_planner/potentials/Expander.hpp>
#include <srsnode_navigation/global_planner/potentials/PotentialCalculator.hpp>
#include <srsnode_navigation/global_planner/potentials/Traceback.hpp>

namespace srs {

class AStarPotentials
{
public:
    using PolygonType = std::vector<geometry_msgs::Point>;
    using QueueMapType = std::unordered_map<std::string, PolygonType>;

    struct SearchParameters
    {
        SearchParameters() :
            useQuadratic(false),
            useGridPath(true),
            allowUnknown(false),
            lethalCost(253),
            neutralCost(50),
            weightRatio(100),
            logicalCostRatio(100)
        {}

        bool useQuadratic;
        bool useGridPath;
        bool allowUnknown;
        unsigned int lethalCost;
        unsigned int neutralCost;
        unsigned int weightRatio;
        unsigned int logicalCostRatio;
    };

    AStarPotentials(LogicalMap* logicalMap, costmap_2d::Costmap2D* costMap, QueueMapType queuesMap);

    bool calculatePath(SearchParameters searchParams,
        double start_x, double start_y,
        double end_x, double end_y,
        std::vector<std::pair<float, float>>& path,
        float*& potentials);

    void mapToWorld(double mx, double my, double& wx, double& wy);

    bool worldToMap(double wx, double wy, double& mx, double& my);

private:

    void addMapBorder();

    void clearQueues();

    costmap_2d::Costmap2D* costMap_		{ nullptr };

    LogicalMap* logicalMap_			{ nullptr };

    PotentialCalculator* potentialCalculator_	{ nullptr };
    Traceback* pathBuilder_			{ nullptr };

    Expander* stateExpander_			{ nullptr };

    QueueMapType queuesMap_			{ };
};

}
