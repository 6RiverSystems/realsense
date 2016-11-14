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
    BaseMap(Grid2d* grid, double resolution);
    virtual ~BaseMap();

    virtual Grid2d::BaseType getCost(unsigned int c, unsigned int r) const = 0;

    int getHeightCells() const
    {
        return grid_->getHeight();
    }

    double getHeightM() const
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

    void getWeights(unsigned int c, unsigned int r,
        Grid2d::BaseType& north, Grid2d::BaseType& east,
        Grid2d::BaseType& south, Grid2d::BaseType& west) const
    {
        Grid2d::Location location(c, r);
        grid_->getWeights(location, north, east, south, west);
    }

    int getWidthCells() const
    {
        return grid_->getWidth();
    }

    double getWidthM() const
    {
        return widthM_;
    }

    bool isWithinBounds(unsigned int c, unsigned int r) const
    {
        Grid2d::Location location(c, r);
        return grid_->isWithinBounds(location);
    }

    virtual void maxCost(unsigned int c, unsigned int r, Grid2d::BaseType cost) = 0;

    friend bool operator==(const BaseMap& lhs, const BaseMap& rhs);

    friend bool operator!=(const BaseMap& lhs, const BaseMap& rhs)
    {
        return !(lhs == rhs);
    }

    virtual void setCost(unsigned int c, unsigned int r, Grid2d::BaseType cost) = 0;
    virtual void setObstruction(unsigned int c, unsigned int r) = 0;

    void transformCells2M(unsigned int cells, double& measurement) const
    {
        // The precision is down to 1mm
        measurement = round(static_cast<double>(cells) * resolution_ * 1e3) / 1e3;
    }

    void transformCells2M(unsigned int c, unsigned int r, double& x, double& y) const
    {
        transformCells2M(c, x);
        transformCells2M(r, y);
    }

    void transformCells2M(unsigned int c, unsigned int r, Pose<>& p) const
    {
        transformCells2M(c, p.x);
        transformCells2M(r, p.y);
    }

    void transformM2Cells(double mesurement, unsigned int& cells) const
    {
        cells = static_cast<unsigned int>(round(mesurement / resolution_));
    }

    void transformM2Cells(double x, double y, unsigned int& c, unsigned int& r) const
    {
        transformM2Cells(x, c);
        transformM2Cells(y, r);
    }

    void transformM2Cells(Pose<> p, unsigned int& c, unsigned int& r) const
    {
        transformM2Cells(p.x, c);
        transformM2Cells(p.y, r);
    }

private:

    Grid2d* grid_;

    double heightM_;

    Pose<> origin_;

    double resolution_;

    bool userSpecifiedGrid_;

    double widthM_;
};

} // namespace srs
