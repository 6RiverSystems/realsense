/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <vector>
#include <algorithm>

#include <srsnode_navigation/global_planner/PotentialCalculator.hpp>
#include <srsnode_navigation/global_planner/SrsPlannerPotentials.hpp>

namespace srs {

class AStarCore
{
public:
    AStarCore(costmap_2d::Costmap2D* costmap);

    bool calculatePath(
        unsigned int start_x, unsigned int start_y,
        unsigned int end_x, unsigned int end_y,
        const geometry_msgs::PoseStamped& goal,
        std::vector<geometry_msgs::PoseStamped>& plan);

private:
    void outlineMap(unsigned char* costarr, int nx, int ny, unsigned char value);

    float* potential_array_;

    costmap_2d::Costmap2D* costmap_;

    PotentialCalculator* p_calc_;
    Expander* planner_;
    Traceback* path_maker_;
    OrientationFilter* orientation_filter_;
};

}
