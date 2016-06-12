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
using namespace srs;

#include <srslib_test/utils/MemoryWatch.hpp>

TEST(Test_Map, Reconstruction)
{
    test::MemoryWatch memoryWatch;

    Map* map = new Map();
    map->load("/home/fsantini/projects/repos/ros/srs_sites/src/srsc_empty/map/empty.yaml");

    map->print();
    cout << map->getGrid()->getCost(Grid2dLocation(1, 0)) << endl;
    cout << *(reinterpret_cast<MapNote*>(map->getGrid()->getNote(Grid2dLocation(1, 0)))) << endl;

    vector<int8_t> costGrid;
    vector<int8_t> notesGrid;

    map->getCostsGrid(costGrid);
    map->getNotesGrid(notesGrid);

    double widthCells = map->getWidthCells();
    double heightCells = map->getHeightCells();
    double resolution = map->getResolution();

    cout << "Memory usage: " << memoryWatch.getMemoryUsage() << endl;
    delete map;

    map = new Map(widthCells, heightCells, resolution);
    map->setGrid(costGrid, notesGrid);

    map->print();
    cout << map->getGrid()->getCost(Grid2dLocation(1, 0)) << endl;
    cout << *(reinterpret_cast<MapNote*>(map->getGrid()->getNote(Grid2dLocation(1, 0)))) << endl;

    // TODO: Reaserch the memory leaks in the YAML library
    cout << "End memory usage: " << memoryWatch.getMemoryUsage() << endl;
    cout << "Zero marker: " << memoryWatch.isZero() << endl;
}
