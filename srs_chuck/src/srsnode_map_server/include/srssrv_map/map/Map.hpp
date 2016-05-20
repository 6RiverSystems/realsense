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

namespace srs {

class Map
{
public:
    Map();

    ~Map();

    unsigned int getHeight()
    {
        return height_;
    }

    void getOccupancyGrid(vector<int8_t>& occupancyGrid);

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

    unsigned int getWidth()
    {
        return width_;
    }

    void load(string filename);

private:
    const unsigned char FLAG_OD = 0x80;
    const unsigned char FLAG_GO_SLOW = 0x40;
    const unsigned char FLAG_NO_ROTATIONS = 0x20;

    void loadConfiguration();
    void loadCosts();

    YAML::Node document_;

    unsigned int height_;

    string mapDocumentFilename_;
    string mapImageFilename_;

    tf::Quaternion origin_;
    tf::Quaternion orientation_;

    double resolution_;

    unsigned int width_;

    Grid2d* grid_;
};

} // namespace srs

#endif // MAP_HPP_
