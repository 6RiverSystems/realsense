/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#include <gtest/gtest.h>
#include <ros/ros.h>

#include <vector>
using namespace std;

#include <srslib_framework/datastructure/graph/grid2d/Grid2d.hpp>
#include <srslib_framework/search/AStar.hpp>
using namespace srs;

#include <srslib_test/datastructure/graph/grid2d/Grid2dUtils.hpp>

TEST(Test_Graph, Grid2dCreation)
{
    Grid2d grid(6, 8);

    test::Grid2dUtils::addRectangleCost(grid, 1, 1, 3, 2, 900);
    test::Grid2dUtils::addRectangleCost(grid, 4, 1, 4, 1, 900);
    test::Grid2dUtils::addRectangleCost(grid, 3, 4, 3, 7, 900);

    ROS_DEBUG_STREAM(grid);
    ROS_DEBUG_STREAM(grid.getCost(Grid2d::LocationType(1, 1)));

    grid.clear();

    ROS_DEBUG_STREAM(grid);
    ROS_DEBUG_STREAM(grid.getCost(Grid2d::LocationType(1, 1)));

}
