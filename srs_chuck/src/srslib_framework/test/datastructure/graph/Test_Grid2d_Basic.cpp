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
using namespace srs;

#include <srslib_test/datastructure/graph/grid2d/Grid2dUtils.hpp>

TEST(Test_Graph, Grid2dCreation)
{
    Grid2d grid(10);

    grid.setCost(Grid2d::Location(1, 1), 10);
    grid.setCost(Grid2d::Location(3, 3), 30);

    cout << grid << endl;

    grid.setAggregateSize(1, 1);

    cout << grid << endl;

    grid.setCost(Grid2d::Location(3, 3), 22);
    cout << grid << endl;

    grid.setCost(Grid2d::Location(2, 2), 10);
    cout << grid << endl;

    grid.addCost(Grid2d::Location(1, 1), 10);
    cout << grid << endl;
}
