/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/robotics/Pose.hpp>
#include <srslib_framework/localization/map/BaseMetadata.hpp>

namespace srs {

using namespace std;

struct OccupancyMetadata : public BaseMetadata
{
public:
    OccupancyMetadata() :
        occupancyFilename(""),
        thresholdOccupied(0.9),
        thresholdFree(0.1),
        negate(false)
    {}

    string occupancyFilename;

    bool negate;

    double thresholdFree;
    double thresholdOccupied;
};

} // namespace srs
