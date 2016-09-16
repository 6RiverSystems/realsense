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
using namespace srs;

#include <srslib_test/utils/MemoryWatch.hpp>

TEST(Test_Map, Reconstruction)
{
    test::MemoryWatch memoryWatch;

    Map* map = new Map();
    map->load("/tmp/srslib_framework/data/empty.yaml");

    ROS_DEBUG_STREAM(*map);

    ROS_DEBUG_STREAM(map->getGrid()->getCost(Grid2dLocation(1, 0)));
    ROS_DEBUG_STREAM(*(reinterpret_cast<MapNote*>(map->getGrid()->getNote(Grid2dLocation(1, 0)))));

    vector<int8_t> costGrid;
    vector<int8_t> notesGrid;

    map->getCostsGrid(costGrid);
    map->getNotesGrid(notesGrid);

    double widthCells = map->getWidthCells();
    double heightCells = map->getHeightCells();
    double resolution = map->getResolution();

    ROS_DEBUG_STREAM("Memory usage: " << memoryWatch.getMemoryUsage());
    delete map;

    map = new Map(widthCells, heightCells, resolution);
    map->setGrid(costGrid, notesGrid);

    ROS_DEBUG_STREAM(*map);

    ROS_DEBUG_STREAM(map->getGrid()->getCost(Grid2dLocation(1, 0)));
    ROS_DEBUG_STREAM(*(reinterpret_cast<MapNote*>(map->getGrid()->getNote(Grid2dLocation(1, 0)))));

    // TODO: Reaserch the memory leaks in the YAML library
    ROS_DEBUG_STREAM("End memory usage: " << memoryWatch.getMemoryUsage());
    ROS_DEBUG_STREAM("Zero marker: " << memoryWatch.isZero());
}
