/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#include <gtest/gtest.h>

#include <vector>
using namespace std;

#include <srslib_framework/graph/grid2d/Grid2d.hpp>
#include <srslib_framework/localization/Map.hpp>
#include <srslib_framework/search/AStar.hpp>
#include <srslib_framework/search/SearchPositionNote.hpp>

#include <srslib_test/utils/Print.hpp>
#include <srslib_test/graph/grid2d/Grid2dUtils.hpp>
using namespace srs;

constexpr int GRID_SIZE = 5;

TEST(Test_AStar, WithMap)
{
    Map map;
    map.load("/home/fsantini/projects/repos/ros/srs_sites/src/srsc_6rhq/map/6rhq.yaml");

    AStar<Grid2d> algorithm(map.getGrid());

    Grid2d::LocationType start(29, 19);
    Grid2d::LocationType goal(58, 28);

    algorithm.search(SearchPosition<Grid2d>(start, 0), SearchPosition<Grid2d>(goal, 90));

    vector<SolutionNode<Grid2d>> path = algorithm.getPath(1.0);
    for (auto node : path)
    {
        cout << node << endl;
    }
    cout << endl;
}
