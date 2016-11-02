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
#include <srslib_test/datastructure/graph/grid2d/Grid2dUtils.hpp>
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

TEST(Test_Navigation, SmallMap)
{
    Grid2d grid(GRID_SIZE, GRID_SIZE);
    test::Grid2dUtils::addStaticObstacle(grid, 4, 7, 8, 7);

    grid.setWeights(Grid2d::Location(8, 12), 0, 0, 0, 250);
    grid.setWeights(Grid2d::Location(8, 11), 0, 0, 0, 250);
    grid.setWeights(Grid2d::Location(8, 10), 0, 0, 0, 250);
    grid.setWeights(Grid2d::Location(8, 9), 0, 0, 0, 250);
    grid.setWeights(Grid2d::Location(8, 8), 0, 0, 0, 250);

    grid.setWeights(Grid2d::Location(7, 12), 0, 0, 0, 250);
    grid.setWeights(Grid2d::Location(7, 11), 0, 0, 0, 250);
    grid.setWeights(Grid2d::Location(7, 10), 0, 0, 0, 250);
    grid.setWeights(Grid2d::Location(7, 9), 0, 0, 0, 250);
    grid.setWeights(Grid2d::Location(7, 8), 0, 0, 0, 250);

    grid.setWeights(Grid2d::Location(6, 12), 0, 0, 0, 250);
    grid.setWeights(Grid2d::Location(6, 11), 0, 0, 0, 250);
    grid.setWeights(Grid2d::Location(6, 10), 0, 0, 0, 250);
    grid.setWeights(Grid2d::Location(6, 9), 0, 0, 0, 250);
    grid.setWeights(Grid2d::Location(6, 8), 0, 0, 0, 250);

    grid.setWeights(Grid2d::Location(5, 12), 0, 0, 0, 250);
    grid.setWeights(Grid2d::Location(5, 11), 0, 0, 0, 250);
    grid.setWeights(Grid2d::Location(5, 10), 0, 0, 0, 250);
    grid.setWeights(Grid2d::Location(5, 9), 0, 0, 0, 250);
    grid.setWeights(Grid2d::Location(5, 8), 0, 0, 0, 250);

    grid.setWeights(Grid2d::Location(4, 12), 0, 0, 0, 250);
    grid.setWeights(Grid2d::Location(4, 11), 0, 0, 0, 250);
    grid.setWeights(Grid2d::Location(4, 10), 0, 0, 0, 250);
    grid.setWeights(Grid2d::Location(4, 9), 0, 0, 0, 250);
    grid.setWeights(Grid2d::Location(4, 8), 0, 0, 0, 250);

    LogicalMapFactory logicalMapFactory;
    LogicalMap* logical = logicalMapFactory.fromGrid2d(&grid, 1);

    costmap_2d::Costmap2D* cost2d = MapAdapter::toCostMap2D(logical);

    cout << *logical << endl;

    AStarCore astar(logical, cost2d);
    std::vector<std::pair<float, float>> path;
    float* potentials;

    if (astar.calculatePath(8, 10, 4, 10, path, potentials))
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
