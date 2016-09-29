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
#include <srslib_framework/localization/map/Map.hpp>
#include <srslib_framework/localization/map/MapFactory.hpp>
#include <srslib_framework/search/AStar.hpp>
#include <srslib_framework/localization/map/MapNote.hpp>

#include <srslib_test/utils/MemoryWatch.hpp>
using namespace srs;

TEST(Test_AStar, WithMap)
{
    Grid2d::LocationType start(86, 48);
    Grid2d::LocationType goal(87, 48);

    Map* map = new Map();
    map->load("/tmp/srslib_framework/data/6rhq.yaml");

    vector<int8_t> occupancyMap;
    vector<int8_t> notesGrid;

    MapFactory::map2Occupancy(map, occupancyMap);
    MapFactory::map2Notes(map, notesGrid);

    double widthCells = map->getWidthCells();
    double heightCells = map->getHeightCells();
    double resolution = map->getResolution();

    delete map;

    map = new Map(widthCells, heightCells, resolution);
    map->setGrid(occupancyMap, notesGrid);

    test::MemoryWatch memoryWatch;

    AStar<Grid2d>* algorithm = new AStar<Grid2d>(map->getGrid());

    ROS_DEBUG_STREAM("Found: " <<
        algorithm->search(SearchPosition<Grid2d>(start, 0), SearchPosition<Grid2d>(goal, 0)));

    algorithm->clear();

    delete algorithm;

    ROS_DEBUG_STREAM("Memory usage: " << memoryWatch.getMemoryUsage());
    ROS_DEBUG_STREAM("Memory leaks: " << !memoryWatch.isZero());
}
