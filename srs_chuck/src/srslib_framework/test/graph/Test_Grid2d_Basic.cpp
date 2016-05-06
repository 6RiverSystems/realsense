/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#include <gtest/gtest.h>

#include <vector>
using namespace std;

#include <srslib_framework/graph/grid2d/Grid2d.hpp>
#include <srslib_framework/search/AStar.hpp>
using namespace srs;

#include <srslib_test/graph/grid2d/Grid2dUtils.hpp>

TEST(Test_Graph, Grid2dCreation)
{
    Grid2d grid(6, 8);

    test::Grid2dUtils::addRectangleCost(grid, 1, 1, 3, 2, 900);
    test::Grid2dUtils::addRectangleCost(grid, 4, 1, 4, 1, 900);
    test::Grid2dUtils::addRectangleCost(grid, 3, 4, 3, 7, 900);

    cout << grid << endl;
    cout << grid.getCost(Grid2d::LocationType(1, 1)) << endl;

    grid.clear();

    cout << grid << endl;
    cout << grid.getCost(Grid2d::LocationType(1, 1)) << endl;

}
