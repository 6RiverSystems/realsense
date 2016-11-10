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
#include <srslib_test/utils/Print.hpp>

#include <srsnode_navigation/global_planner/AStarPotentials.hpp>
using namespace srs;

constexpr int GRID_SIZE = 15;

TEST(Test_Navigation, OneWay)
{
    MapStack* mapStack = MapStackFactory::fromJsonFile("data/one-way/one-way.yaml");
    LogicalMap* logical = mapStack->getLogicalMap();

    cout << *logical << endl;

    cout << *(mapStack->getOccupancyMap()) << endl;

    costmap_2d::Costmap2D* cost2d = MapAdapter::map2CostMap2D(mapStack->getOccupancyMap());

    AStarPotentials astar(logical, cost2d);
    std::vector<std::pair<float, float>> path;
    float* potentials;

    if (astar.calculatePath(1, 1, 4, 1, path, potentials))
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
}
