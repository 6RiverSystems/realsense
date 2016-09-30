/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
#include <sstream>
using namespace std;

#include <tf/tf.h>

#include <srslib_framework/graph/grid2d/Grid2d.hpp>
#include <srslib_framework/localization/map/BaseMetadata.hpp>
#include <srslib_framework/localization/map/MapNote.hpp>

namespace srs {

class BaseMap
{
public:
    BaseMap();
    BaseMap(double widthMeters, double heightMeters, double resolution);
    BaseMap(BaseMetadata metadata);

    virtual ~BaseMap();

    int getHeightCells()
    {
        return baseMetadata_.heightCells;
    }

    double getHeightMeters()
    {
        return baseMetadata_.heightM;
    }

    Grid2d* getGrid()
    {
        return grid_;
    }

    void getMapCoordinates(double x, double y, int& c, int& r)
    {
        c = static_cast<int>(round(x / baseMetadata_.resolution));
        r = static_cast<int>(round(y / baseMetadata_.resolution));
    }

    float getResolution()
    {
        return baseMetadata_.resolution;
    }

    int getWidthCells()
    {
        return baseMetadata_.widthCells;
    }

    double getWidthMeters()
    {
        return baseMetadata_.widthM;
    }

    void getWorldCoordinates(int c, int r, double& x, double& y)
    {
        x = static_cast<double>(c) * baseMetadata_.resolution;
        y = static_cast<double>(r) * baseMetadata_.resolution;

        // The precision is down to 1mm
        x = round(x * 1000) / 1000;
        y = round(y * 1000) / 1000;
    }

    virtual void setCost(int c, int r, unsigned int cost) = 0;
    virtual void setObstruction(int c, int r) = 0;

protected:
    BaseMetadata baseMetadata_;

    Grid2d* grid_;
};

} // namespace srs
