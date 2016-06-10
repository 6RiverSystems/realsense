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

TEST(Test_Map, Reconstruction)
{
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

    delete map;

    map = new Map(widthCells, heightCells, resolution);
    map->setGrid(costGrid, notesGrid);

    map->print();
    cout << map->getGrid()->getCost(Grid2dLocation(1, 0)) << endl;
    cout << *(reinterpret_cast<MapNote*>(map->getGrid()->getNote(Grid2dLocation(1, 0)))) << endl;
}
