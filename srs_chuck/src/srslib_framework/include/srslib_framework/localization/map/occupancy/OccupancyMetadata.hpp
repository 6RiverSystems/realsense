/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/robotics/Pose.hpp>

namespace srs {

using namespace std;

struct OccupancyMetadata
{
public:
    OccupancyMetadata() :
        loadTime(0),
        heightCells(0),
        widthCells(0),
        heightMm(0.0),
        widthMm(0.0),
        origin(Pose<>::INVALID),
        resolution(0.0),
        occupancyFilename(""),
        thresholdOccupied(0.9),
        thresholdFree(0.1),
        negate(false)
    {}

    int heightCells;
    double heightMm;

    double loadTime;

    bool negate;

    string occupancyFilename;
    Pose<> origin;

    double resolution;

    double thresholdFree;
    double thresholdOccupied;

    int widthCells;
    double widthMm;
};

} // namespace srs
