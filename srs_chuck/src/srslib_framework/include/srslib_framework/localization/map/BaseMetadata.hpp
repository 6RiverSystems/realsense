/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/robotics/Pose.hpp>

namespace srs {

using namespace std;

struct BaseMetadata
{
public:
    BaseMetadata() :
        loadTime(0),
        heightCells(0),
        widthCells(0),
        heightM(0.0),
        widthM(0.0),
        origin(Pose<>::INVALID),
        resolution(0.0)
    {}

    double loadTime;

    int heightCells;
    double heightM;

    Pose<> origin;

    double resolution;

    int widthCells;
    double widthM;
};

} // namespace srs
