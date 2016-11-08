/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <vector>
#include <algorithm>

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>

#include <srslib_framework/localization/map/logical/LogicalMap.hpp>

#include <srsnode_navigation/global_planner/Expander.hpp>
#include <srsnode_navigation/global_planner/PotentialCalculator.hpp>
#include <srsnode_navigation/global_planner/Traceback.hpp>

namespace srs {

class AStarPotentials
{
public:
    AStarPotentials(LogicalMap* logicalMap, costmap_2d::Costmap2D* costMap);

    bool calculatePath(
        double start_x, double start_y,
        double end_x, double end_y,
        std::vector<std::pair<float, float>>& path,
        float*& potentials);

    void mapToWorld(double mx, double my, double& wx, double& wy);

    bool worldToMap(double wx, double wy, double& mx, double& my);

private:
    void addMapBorder();

    costmap_2d::Costmap2D* costMap_;

    LogicalMap* logicalMap_;

    Traceback* pathBuilder_;
    PotentialCalculator* potentialCalculator_;

    Expander* stateExpander_;
};

}
