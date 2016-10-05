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
    BaseMap(double widthM, double heightM, double resolution);
    virtual ~BaseMap();

    virtual unsigned int getCost(unsigned int c, unsigned int r) const = 0;

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

    double getResolution() const
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

    virtual void setCost(unsigned int c, unsigned int r, unsigned int cost) = 0;
    virtual void setObstruction(unsigned int c, unsigned int r) = 0;

    void transformCells2M(unsigned int cells, double& measurement)
    {
        // The precision is down to 1mm
        double r = static_cast<double>(cells) * resolution_;
        measurement = round(r * 1000) / 1000;
    }

    void transformCells2M(unsigned int c, unsigned int r, double& x, double& y)
    {
        transformCells2M(c, x);
        transformCells2M(r, y);
    }

    void transformCells2M(unsigned int c, unsigned int r, Pose<>& p)
    {
        transformCells2M(c, p.x);
        transformCells2M(r, p.y);
    }

    void transformM2Cells(double mesurement, unsigned int& cells)
    {
        cells = static_cast<unsigned int>(round(mesurement / resolution_));
    }

    void transformM2Cells(double x, double y, unsigned int& c, unsigned int& r)
    {
        transformM2Cells(x, c);
        transformM2Cells(y, r);
    }

    void transformM2Cells(Pose<> p, unsigned int& c, unsigned int& r)
    {
        transformM2Cells(p.x, c);
        transformM2Cells(p.y, r);
    }

private:

    double heightM_;

    Pose<> origin_;

    Grid2d* grid_;

    double resolution_;

    double widthM_;
};

} // namespace srs
