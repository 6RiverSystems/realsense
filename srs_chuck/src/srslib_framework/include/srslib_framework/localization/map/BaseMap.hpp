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
#include <srslib_framework/robotics/Pose.hpp>

namespace srs {

class BaseMap
{
public:
    BaseMap(unsigned int widthCells, unsigned int heightCells, double resolution);
    virtual ~BaseMap();

    int getHeightCells() const
    {
        return grid_->getHeight();
    }

    double getHeightMeters() const
    {
        return heightM_;
    }

    Grid2d* getGrid() const
    {
        return grid_;
    }

    void getMapCoordinates(double x, double y, int& c, int& r)
    {
        c = static_cast<int>(round(x / resolution_));
        r = static_cast<int>(round(y / resolution_));
    }

    float getResolution() const
    {
        return resolution_;
    }

    int getWidthCells() const
    {
        return grid_->getWidth();
    }

    double getWidthMeters() const
    {
        return widthM_;
    }

    void getWorldCoordinates(int c, int r, double& x, double& y)
    {
        x = static_cast<double>(c) * resolution_;
        y = static_cast<double>(r) * resolution_;

        // The precision is down to 1mm
        x = round(x * 1000) / 1000;
        y = round(y * 1000) / 1000;
    }

    virtual void setCost(int c, int r, unsigned int cost) = 0;
    virtual void setObstruction(int c, int r) = 0;

protected:

    double heightM_;

    Pose<> origin_;

    Grid2d* grid_;

    double resolution_;

    double widthM_;
};

} // namespace srs
