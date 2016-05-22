/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef MAP_HPP_
#define MAP_HPP_

#include <string>
using namespace std;

#include <yaml-cpp/yaml.h>

#include <tf/LinearMath/Quaternion.h>

#include <srslib_framework/graph/grid2d/Grid2d.hpp>
#include <srslib_framework/search/SearchPositionNote.hpp>

namespace srs {

class Map
{
public:
    Map();
    Map(double widthMeters, double heightMeters, double resolution);

    ~Map();

    int getHeightCells()
    {
        return heightC_;
    }

    double getHeightMeters()
    {
        return heightM_;
    }

    Grid2d* getGrid()
    {
        return grid_;
    }

    void getMapCoordinates(double x, double y, int c, int r);
    void getCostsGrid(vector<int8_t>& costGrid);
    void getNotesGrid(vector<int8_t>& notesGrid);

    tf::Quaternion getOrigin()
    {
        return origin_;
    }

    tf::Quaternion getOrientation()
    {
        return orientation_;
    }

    float getResolution()
    {
        return resolution_;
    }

    int getWidthCells()
    {
        return widthC_;
    }

    double getWidthMeters()
    {
        return widthM_;
    }

    void load(string filename);

    void setGrid(vector<int8_t>& costGrid, vector<int8_t>& notesGrid);

private:
    const unsigned char FLAG_OD = 0x80;
    const unsigned char FLAG_GO_SLOW = 0x40;
    const unsigned char FLAG_NO_ROTATIONS = 0x20;

    SearchPositionNote* createNote(unsigned char flags, SearchPositionNote* note = nullptr);

    void loadConfiguration();
    void loadCosts();

    YAML::Node document_;

    int heightC_;
    double heightM_;

    string mapDocumentFilename_;
    string mapImageFilename_;

    tf::Quaternion origin_;
    tf::Quaternion orientation_;

    double resolution_;

    int widthC_;
    double widthM_;

    Grid2d* grid_;
};

} // namespace srs

#endif // MAP_HPP_
