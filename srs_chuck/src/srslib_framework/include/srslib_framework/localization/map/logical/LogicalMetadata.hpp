/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

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
        heightMm(0.0),
        widthMm(0.0),
        origin(Pose<>::INVALID),
        resolution(0.0),
        logicalFilename("")
    {}

    int heightCells;
    double heightMm;

    double loadTime;
    string logicalFilename;

    Pose<> origin;

    double resolution;

    int widthCells;
    double widthMm;
};

} // namespace srs
