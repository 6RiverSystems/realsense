/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#include <gtest/gtest.h>
#include <ros/ros.h>

#include <vector>
using namespace std;

#include <srslib_framework/graph/grid2d/Grid2d.hpp>
#include <srslib_framework/search/AStar.hpp>
#include <srslib_framework/localization/map/MapNote.hpp>
using namespace srs;

constexpr int GRID_SIZE = 5;

TEST(Test_AStar, SearchCreation)
{
    Grid2d grid(GRID_SIZE, GRID_SIZE);

    // TODO: Add something to the map: fix the following
//    test::Grid2dUtils::addLocationCost(grid, 0, 0, 0, &MapNote::NO_ROTATIONS);
//    test::Grid2dUtils::addLocationCost(grid, 0, 1, 0);
//
//    test::Grid2dUtils::addRectangleCost(grid, 1, 1, 3, 2, 1000, &MapNote::STATIC_OBSTACLE);
//    test::Grid2dUtils::addLocationCost(grid, 4, 1, 1000, &MapNote::STATIC_OBSTACLE);
//    test::Grid2dUtils::addRectangleCost(grid, 3, 4, 3, 7, 1000, &MapNote::STATIC_OBSTACLE);

    AStar<Grid2d> algorithm(&grid);

    Grid2d::LocationType start(0, 0);
    Grid2d::LocationType goal(GRID_SIZE - 1, GRID_SIZE - 1);

    algorithm.search(SearchPosition<Grid2d>(start, 0), SearchPosition<Grid2d>(goal, 0));
}
