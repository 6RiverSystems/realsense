/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#include <gtest/gtest.h>

#include <vector>
using namespace std;

#include <srslib_framework/localization/map/MapAdapter.hpp>
#include <srslib_framework/localization/map/MapStack.hpp>
#include <srslib_framework/localization/map/MapStackFactory.hpp>
//#include <srslib_framework/datastructure/graph/grid2d/Grid2d.hpp>
//#include <srslib_test/datastructure/graph/grid2d/Grid2dUtils.hpp>
#include <srsnode_navigation/global_planner/AStarCore.hpp>
using namespace srs;

constexpr int GRID_SIZE = 15;

void printPotentials(float* potentials)
{
    cout << setw(8) << "  ";
    for (int col = 0; col < GRID_SIZE; ++col) {
        cout << setw(8) << col << " ";
    }

    cout << endl;
    for (int row = GRID_SIZE - 1; row >= 0; row--)
    {
        cout << setw(8) << row << " ";
        for (int col = 0; col < GRID_SIZE; ++col)
        {
            float p = *(potentials + (row * GRID_SIZE) + col);
            cout << setw(8) << p << " ";
        }
        cout << endl;
    }
}

TEST(Test_Navigation, OneWay)
{
    MapStack* mapStack = MapStackFactory::fromJsonFile("data/one-way/one-way.yaml");
    LogicalMap* logical = mapStack->getLogicalMap();

    cout << *logical << endl;

    costmap_2d::Costmap2D* cost2d = MapAdapter::map2CostMap2D(logical);

    AStarCore astar(logical, cost2d);
    std::vector<std::pair<float, float>> path;
    float* potentials;

    if (astar.calculatePath(1, 7, 14, 7, path, potentials))
    {
        for (auto step : path)
        {
            cout << step.first << " - " << step.second << endl;
        }

        printPotentials(potentials);
    }
    else
    {
        cout << "Path not found" << endl;
    }
}
