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

#include <srslib_framework/datastructure/graph/grid2d/Grid2d.hpp>
#include <srslib_framework/robotics/Pose.hpp>

namespace srs {

class BaseMap
{
public:
    BaseMap(unsigned int widthCells, unsigned int heightCells, double resolution);
    BaseMap(double widthM, double heightM, double resolution);
    virtual ~BaseMap();

    virtual void addCost(unsigned int c, unsigned int r, int cost) = 0;

    virtual int getCost(unsigned int c, unsigned int r) const = 0;

    int getHeightCells() const
    {
        return grid_->getHeight();
    }

    double getHeightMm() const
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

    double getWidthMm() const
    {
        return widthM_;
    }

    virtual void setCost(unsigned int c, unsigned int r, int cost) = 0;
    virtual void setObstruction(unsigned int c, unsigned int r) = 0;

    void transformCells2Mm(unsigned int cells, double& measurement)
    {
        // The precision is down to 1mm
        double r = static_cast<double>(cells) * resolution_;
        measurement = round(r);
    }

    void transformCells2Mm(unsigned int c, unsigned int r, double& x, double& y)
    {
        transformCells2Mm(c, x);
        transformCells2Mm(r, y);
    }

    void transformCells2Mm(unsigned int c, unsigned int r, Pose<>& p)
    {
        transformCells2Mm(c, p.x);
        transformCells2Mm(r, p.y);
    }

    void transformMm2Cells(double mesurement, unsigned int& cells)
    {
        cells = static_cast<unsigned int>(round(mesurement / resolution_));
    }

    void transformMm2Cells(double x, double y, unsigned int& c, unsigned int& r)
    {
        transformMm2Cells(x, c);
        transformMm2Cells(y, r);
    }

    void transformMm2Cells(Pose<> p, unsigned int& c, unsigned int& r)
    {
        transformMm2Cells(p.x, c);
        transformMm2Cells(p.y, r);
    }

private:

    Grid2d* grid_;

    double heightM_;

    Pose<> origin_;

    double resolution_;

    double widthM_;
};

} // namespace srs
