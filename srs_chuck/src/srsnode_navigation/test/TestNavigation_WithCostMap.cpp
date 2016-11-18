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

TEST(Test_Navigation, WithCostMap)
{
    MapStack* mapStack = MapStackFactory::fromJsonFile("data/6rshq_oneway/6rshq_oneway.yaml");
    LogicalMap* logical = mapStack->getLogicalMap();

    cout << *(mapStack->getOccupancyMap()) << endl;

    costmap_2d::Costmap2D* cost2d = MapAdapter::map2CostMap2D(mapStack->getOccupancyMap());

    AStarPotentials astar(logical, cost2d);
    std::vector<std::pair<float, float>> path;
    float* potentials;

    if (astar.calculatePath(23.4168, 14.9508, 24.5957, 15.3492, path, potentials))
    {
        for (auto step : path)
        {
            cout << step.first << " - " << step.second << endl;
        }

        cout << test::Print::printToString(potentials,
            cost2d->getSizeInCellsX(), cost2d->getSizeInCellsY(),
            1e10f);
    }
    else
    {
        cout << "Path not found" << endl;
    }
}
