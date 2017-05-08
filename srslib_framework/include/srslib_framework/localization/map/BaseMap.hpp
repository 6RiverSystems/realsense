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

#include <srslib_framework/datastructure/graph/grid2d/BaseGrid2d.hpp>
#include <srslib_framework/datastructure/Location.hpp>
#include <srslib_framework/math/MeasurementMath.hpp>
#include <srslib_framework/math/PoseMath.hpp>
#include <srslib_framework/robotics/Pose.hpp>

namespace srs {

class BaseMap
{
public:
    BaseMap(BaseGrid2d* grid, double widthM, double heightM, double resolution, Pose<> origin);
    virtual ~BaseMap();

    int getHeightCells() const
    {
        return grid_->getHeight();
    }

    double getHeightM() const
    {
        return heightM_;
    }

    Pose<> getOrigin() const
    {
        return origin_;
    }

    double getResolution() const
    {
        return resolution_;
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
        Location location(cCells, rCells);
        return grid_->isWithinBounds(location);
    }

    friend bool operator==(const BaseMap& lhs, const BaseMap& rhs);

    friend bool operator!=(const BaseMap& lhs, const BaseMap& rhs)
    {
        return !(lhs == rhs);
    }

    void transformCells2M(unsigned int cCells, unsigned int rCells, double& x, double& y) const
    {
        x = MeasurementMath::cells2M(cCells, resolution_) + origin_.x;
        y = MeasurementMath::cells2M(rCells, resolution_) + origin_.y;
    }

    void transformCells2M(unsigned int cCells, unsigned int rCells, Pose<>& p) const
    {
        p.x = MeasurementMath::cells2M(cCells, resolution_);
        p.y = MeasurementMath::cells2M(rCells, resolution_);

        p = PoseMath::add(p, origin_);
    }

    void transformM2Cells(double x, double y, unsigned int& c, unsigned int& r) const
    {
        c = MeasurementMath::m2Cells(x - origin_.x, resolution_);
        r = MeasurementMath::m2Cells(y - origin_.y, resolution_);
    }

    void transformM2Cells(Pose<> p, unsigned int& c, unsigned int& r) const
    {
        c = MeasurementMath::m2Cells(p.x - origin_.x, resolution_);
        r = MeasurementMath::m2Cells(p.y - origin_.y, resolution_);
    }

protected:
    BaseGrid2d* grid_;

private:
    double resolution_;

    double widthM_;

    double heightM_;

    Pose<> origin_;
};

} // namespace srs
