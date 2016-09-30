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

struct LogicalMetadata : public BaseMetadata
{
public:
    LogicalMetadata() :
        logicalFilename("")
    {}

    string logicalFilename;
};

} // namespace srs
