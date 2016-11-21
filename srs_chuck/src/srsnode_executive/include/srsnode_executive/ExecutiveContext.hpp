/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/localization/map/MapStack.hpp>
#include <srslib_framework/robotics/Pose.hpp>

namespace srs {

struct ExecutiveContext
{
    enum DirectionEnum {
        ENTERING, EXITING, RESIDING
    };

    LogicalMap::LabeledAreaMapType currentLabeledAreas;
    vector<pair<LogicalMap::LabeledArea, DirectionEnum>> activeLabeledAreas;

    MapStack* mapStack;

    Pose<> robotPose;
};

} // namespace srs
