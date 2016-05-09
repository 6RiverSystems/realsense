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
#include <srslib_framework/search/SearchPositionNote.hpp>

#include <srslib_test/utils/Print.hpp>
#include <srslib_test/graph/grid2d/Grid2dUtils.hpp>
using namespace srs;

TEST(Test_AStar, SearchCreation)
{
    Grid2d grid(600, 600);

    SearchPositionNote permanentObstacle(numeric_limits<unsigned int>::max());
    SearchPositionNote noRotationNote(true);

    test::Grid2dUtils::addLocationCost(grid, 0, 0, 0, &noRotationNote);
    test::Grid2dUtils::addLocationCost(grid, 0, 1, 0);

    test::Grid2dUtils::addRectangleCost(grid, 1, 1, 3, 2, 1000, &permanentObstacle);
    test::Grid2dUtils::addLocationCost(grid, 4, 1, 1000, &permanentObstacle);
    test::Grid2dUtils::addRectangleCost(grid, 3, 4, 3, 7, 1000, &permanentObstacle);

    AStar<Grid2d> algorithm(grid);

    Grid2d::LocationType start(0, 0);
    Grid2d::LocationType goal(599, 599);

    algorithm.search(SearchPosition<Grid2d>(start, 180), SearchPosition<Grid2d>(goal, 270));
    vector<SolutionNode<Grid2d>> path = algorithm.getPath();

    for (auto node : path)
    {
        cout << node << endl;
    }
    cout << endl;
}
