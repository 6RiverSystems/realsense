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
#include <srslib_framework/math/PoseMath.hpp>
#include <srslib_framework/robotics/Pose.hpp>

namespace srs {

class BaseMap
{
public:
    BaseMap(unsigned int widthCells, unsigned int heightCells, double resolution, Pose<> origin);
    BaseMap(double widthM, double heightM, double resolution, Pose<> origin);
    BaseMap(Grid2d* grid, double resolution, Pose<> origin);
    virtual ~BaseMap();

    void convertCells2M(unsigned int cells, double& m) const
    {
        // The precision is down to 1mm
        m = round(static_cast<double>(cells) * resolution_ * 1e3) / 1e3;
    }

    void convertM2Cells(double m, unsigned int& cells) const
    {
        cells = static_cast<unsigned int>(round(m / resolution_));
    }

    virtual Grid2d::BaseType getCost(unsigned int cCells, unsigned int rCells) const = 0;

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

    Pose<> getOrigin() const
    {
        return origin_;
    }

    double getResolution() const
    {
        return resolution_;
    }

    void getWeights(unsigned int cCells, unsigned int rCells,
        Grid2d::BaseType& north, Grid2d::BaseType& east,
        Grid2d::BaseType& south, Grid2d::BaseType& west) const
    {
        Grid2d::Location location(cCells, rCells);
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

    bool isWithinBounds(unsigned int cCells, unsigned int rCells) const
    {
        Grid2d::Location location(cCells, rCells);
        return grid_->isWithinBounds(location);
    }

    virtual void maxCost(unsigned int cCells, unsigned int rCells, Grid2d::BaseType cost) = 0;

    friend bool operator==(const BaseMap& lhs, const BaseMap& rhs);

    friend bool operator!=(const BaseMap& lhs, const BaseMap& rhs)
    {
        return !(lhs == rhs);
    }

    virtual void setCost(unsigned int cCells, unsigned int rCells, Grid2d::BaseType cost) = 0;
    virtual void setObstacle(unsigned int cCells, unsigned int rCells) = 0;

    void transformCells2M(unsigned int cCells, unsigned int rCells, double& x, double& y) const
    {
        convertCells2M(cCells, x);
        x += origin_.x;

        convertCells2M(rCells, y);
        y += origin_.y;
    }

    void transformCells2M(unsigned int cCells, unsigned int rCells, Pose<>& p) const
    {
        convertCells2M(cCells, p.x);
        convertCells2M(rCells, p.y);

        p = PoseMath::add(p, origin_);
    }

    void transformM2Cells(double x, double y, unsigned int& c, unsigned int& r) const
    {
        convertM2Cells(x - origin_.x, c);
        convertM2Cells(y - origin_.y, r);
    }

    void transformM2Cells(Pose<> p, unsigned int& c, unsigned int& r) const
    {
        convertM2Cells(p.x - origin_.x, c);
        convertM2Cells(p.y - origin_.y, r);
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
