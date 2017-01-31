/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/math/PoseMath.hpp>
#include <srslib_framework/math/MeasurementMath.hpp>
#include <srslib_framework/robotics/Pose.hpp>

namespace srs {

using namespace std;

struct OccupancyMetadata
{
public:
    OccupancyMetadata() :
        OccupancyMetadata(0, 0, 0, 0.0, 0.0, Pose<>::INVALID, 0.0, "", 0.9, 0.1, false)
    {}

    OccupancyMetadata(double loadTime,
            unsigned int widthCells, unsigned int heightCells,
            Pose<> origin, double resolution,
            string occupancyFilename,
            double thresholdOccupied, double thresholdFree, bool negate) :
                OccupancyMetadata(loadTime,
                    widthCells, heightCells,
                    MeasurementMath::cells2M(widthCells, resolution),
                    MeasurementMath::cells2M(heightCells, resolution),
                    origin, resolution,
                    occupancyFilename,
                    thresholdOccupied, thresholdFree,
                    negate)
    {}

    OccupancyMetadata(double loadTime,
            double widthM, double heightM,
            Pose<> origin, double resolution,
            string occupancyFilename,
            double thresholdOccupied, double thresholdFree, bool negate) :
                OccupancyMetadata(loadTime,
                    MeasurementMath::m2Cells(widthM, resolution),
                    MeasurementMath::m2Cells(heightM, resolution),
                    widthM, heightM,
                    origin, resolution,
                    occupancyFilename,
                    thresholdOccupied, thresholdFree,
                    negate)
    {}

    OccupancyMetadata(const OccupancyMetadata& other) :
        loadTime(other.loadTime),
        heightCells(other.heightCells),
        widthCells(other.widthCells),
        heightM(other.heightM),
        widthM(other.widthM),
        origin(other.origin),
        resolution(other.resolution),
        occupancyFilename(other.occupancyFilename),
        thresholdOccupied(other.thresholdOccupied),
        thresholdFree(other.thresholdFree),
        negate(other.negate)
    {}

    friend bool operator==(const OccupancyMetadata& lhs, const OccupancyMetadata& rhs)
    {
        return lhs.heightCells == rhs.heightCells &&
            BasicMath::equal(lhs.heightM, rhs.heightM, 0.001) &&
            BasicMath::equal(lhs.loadTime, rhs.loadTime, 1e-10) &&
            lhs.negate == rhs.negate &&
            lhs.occupancyFilename == rhs.occupancyFilename &&
            PoseMath::equal(lhs.origin, rhs.origin) &&
            BasicMath::equal(lhs.resolution, rhs.resolution, 0.001) &&
            BasicMath::equal(lhs.thresholdFree, rhs.thresholdFree, 0.001) &&
            BasicMath::equal(lhs.thresholdOccupied, rhs.thresholdOccupied, 0.001) &&
            lhs.widthCells == rhs.widthCells &&
            BasicMath::equal(lhs.widthM, rhs.widthM, 0.001);
    }

protected:
    OccupancyMetadata(double loadTime,
            unsigned int widthCells, unsigned int heightCells,
            double widthM, double heightM,
            Pose<> origin, double resolution,
            string occupancyFilename,
            double thresholdOccupied, double thresholdFree, bool negate) :
        loadTime(loadTime),
        widthCells(widthCells), heightCells(heightCells),
        widthM(widthM), heightM(heightM),
        origin(origin), resolution(resolution),
        occupancyFilename(occupancyFilename),
        thresholdOccupied(thresholdOccupied), thresholdFree(thresholdFree),
        negate(negate)
    {}

public:
    unsigned int heightCells;
    double heightM;

    double loadTime;

    bool negate;

    string occupancyFilename;
    Pose<> origin;

    double resolution;

    double thresholdFree;
    double thresholdOccupied;

    unsigned int widthCells;
    double widthM;
};

} // namespace srs
