/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#include <gtest/gtest.h>

#include <vector>
using namespace std;

#include <srslib_framework/localization/map/MapAdapter.hpp>
#include <srslib_framework/localization/map/logical/LogicalMapFactory.hpp>
#include <srslib_framework/localization/map/occupancy/OccupancyMapFactory.hpp>
#include <srslib_framework/datastructure/graph/grid2d/Grid2d.hpp>
#include <srslib_framework/robotics/Pose.hpp>
#include <srslib_test/datastructure/graph/grid2d/Grid2dUtils.hpp>
#include <srslib_test/utils/Print.hpp>
#include <srsnode_navigation/global_planner/AStarPotentials.hpp>
using namespace srs;

constexpr int GRID_SIZE = 15;

TEST(Test_Navigation_AStarPotentials, SmallSearch)
{
    Grid2d grid(GRID_SIZE, GRID_SIZE);
    //test::Grid2dUtils::addObstacle(grid, 4, 7, 8, 7);
    test::Grid2dUtils::addRectanglePayload(grid, 0, 0, GRID_SIZE - 1, GRID_SIZE - 1, 0);
    //test::Grid2dUtils::addWeights(grid, 4, 8, 8, 12, 0, 0, 0, Grid2d::PAYLOAD_MAX);

    LogicalMapFactory logicalMapFactory;
    LogicalMap* logical = logicalMapFactory.fromGrid2d(&grid, 1, Pose<>::ZERO);

    OccupancyMapFactory occupancyMapFactory;
    OccupancyMap* occupancy = occupancyMapFactory.fromGrid2d(&grid, 1, Pose<>::ZERO);
    costmap_2d::Costmap2D* cost2d = MapAdapter::map2CostMap2D(occupancy);

    cout << *logical << endl;
    cout << *occupancy << endl;

    AStarPotentials astar(logical, cost2d);
    std::vector<std::pair<float, float>> path;
    float* potentials;

    if (astar.calculatePath(AStarPotentials::SearchParameters(), 0, 0, 0, 3, path, potentials))
    {
        for (auto step : path)
        {
            cout << step.first << " - " << step.second << endl;
        }

        cout << test::Print::printToString(potentials, GRID_SIZE, GRID_SIZE);
    }
    else
    {
        cout << "Path not found" << endl;
    }
    cout << test::Print::printToString(potentials, GRID_SIZE, GRID_SIZE);
}
