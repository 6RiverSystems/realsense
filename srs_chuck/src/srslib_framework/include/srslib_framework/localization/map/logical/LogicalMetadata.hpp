/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/math/BasicMath.hpp>
#include <srslib_framework/math/PoseMath.hpp>
#include <srslib_framework/math/MeasurementMath.hpp>
#include <srslib_framework/robotics/Pose.hpp>

namespace srs {

using namespace std;

struct LogicalMetadata
{
public:
    LogicalMetadata() :
        LogicalMetadata(0, 0, 0.0, 0.0, Pose<>::INVALID, 0.0, "")
    {}

    LogicalMetadata(unsigned int widthCells, unsigned int heightCells,
            Pose<> origin, double resolution,
            string logicalFilename) :
                LogicalMetadata(widthCells, heightCells,
                    MeasurementMath::cells2M(widthCells, resolution),
                    MeasurementMath::cells2M(heightCells, resolution),
                    origin, resolution,
                    logicalFilename)
    {}

    LogicalMetadata(double widthM, double heightM,
            Pose<> origin, double resolution,
            string logicalFilename) :
                LogicalMetadata(MeasurementMath::m2Cells(widthM, resolution),
                    MeasurementMath::m2Cells(heightM, resolution),
                    widthM, heightM,
                    origin, resolution, logicalFilename)
    {}

    LogicalMetadata(const LogicalMetadata& other) :
        heightCells(other.heightCells),
        widthCells(other.widthCells),
        heightM(other.heightM),
        widthM(other.widthM),
        origin(other.origin),
        resolution(other.resolution),
        logicalFilename(other.logicalFilename)
    {}

    friend bool operator==(const LogicalMetadata& lhs, const LogicalMetadata& rhs)
    {
        return lhs.heightCells == rhs.heightCells &&
            BasicMath::equal(lhs.heightM, rhs.heightM, 0.001) &&
            lhs.logicalFilename == rhs.logicalFilename &&
            PoseMath::equal(lhs.origin, rhs.origin) &&
            BasicMath::equal(lhs.resolution, rhs.resolution, 0.001) &&
            lhs.widthCells == rhs.widthCells &&
            BasicMath::equal(lhs.widthM, rhs.widthM, 0.001);
    }

protected:
    LogicalMetadata(unsigned int widthCells, unsigned int heightCells,
            double widthM, double heightM,
            Pose<> origin, double resolution,
            string logicalFilename) :
        widthCells(heightCells), heightCells(widthCells),
        widthM(widthM), heightM(heightM),
        origin(origin), resolution(resolution),
        logicalFilename(logicalFilename)
    {}

public:
    int heightCells;
    double heightM;

    string logicalFilename;

    Pose<> origin;

    double resolution;

    int widthCells;
    double widthM;
};

} // namespace srs
