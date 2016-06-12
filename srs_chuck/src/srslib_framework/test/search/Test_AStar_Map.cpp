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
#include <srslib_framework/localization/MapNote.hpp>

#include <srslib_test/utils/MemoryWatch.hpp>
using namespace srs;

TEST(Test_AStar, WithMap)
{
    Grid2d::LocationType start(86, 48);
    Grid2d::LocationType goal(87, 48);

    Map* map = new Map();
    map->load("/home/fsantini/projects/repos/ros/srs_sites/src/srsc_6rhq/map/6rhq.yaml");

    vector<int8_t> costGrid;
    vector<int8_t> notesGrid;

    map->getCostsGrid(costGrid);
    map->getNotesGrid(notesGrid);

    double widthCells = map->getWidthCells();
    double heightCells = map->getHeightCells();
    double resolution = map->getResolution();

    delete map;

    map = new Map(widthCells, heightCells, resolution);
    map->setGrid(costGrid, notesGrid);

    test::MemoryWatch memoryWatch;

    AStar<Grid2d>* algorithm = new AStar<Grid2d>(map->getGrid());

    cout << "Found: " <<
        algorithm->search(SearchPosition<Grid2d>(start, 0), SearchPosition<Grid2d>(goal, 0)) <<
        endl;

    algorithm->clear();

    delete algorithm;

    Solution<Grid2d> solution = algorithm->getSolution(map->getResolution());
    cout << solution << endl;

    cout << "Memory usage: " << memoryWatch.getMemoryUsage() << endl;
    cout << "Memory leaks: " << !memoryWatch.isZero() << endl;
}
