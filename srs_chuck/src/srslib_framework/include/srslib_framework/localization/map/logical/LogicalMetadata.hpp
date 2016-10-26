/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/math/BasicMath.hpp>
#include <srslib_framework/math/PoseMath.hpp>
#include <srslib_framework/robotics/Pose.hpp>

namespace srs {

using namespace std;

struct LogicalMetadata
{
public:
    LogicalMetadata() :
        loadTime(0),
        heightCells(0),
        widthCells(0),
        heightM(0.0),
        widthM(0.0),
        origin(Pose<>::INVALID),
        resolution(0.0),
        logicalFilename("")
    {}

    friend bool operator==(const LogicalMetadata& lhs, const LogicalMetadata& rhs)
    {
        return lhs.heightCells == rhs.heightCells &&
            BasicMath::equal(lhs.heightM, rhs.heightM, 0.001) &&
            BasicMath::equal(lhs.loadTime, rhs.loadTime, 1e-10) &&
            lhs.logicalFilename == rhs.logicalFilename &&
            PoseMath::equal(lhs.origin, rhs.origin) &&
            BasicMath::equal(lhs.resolution, rhs.resolution, 0.001) &&
            lhs.widthCells == rhs.widthCells &&
            BasicMath::equal(lhs.widthM, rhs.widthM, 0.001);
    }

    int heightCells;
    double heightM;

    double loadTime;
    string logicalFilename;

    Pose<> origin;

    double resolution;

    int widthCells;
    double widthM;
};

} // namespace srs
