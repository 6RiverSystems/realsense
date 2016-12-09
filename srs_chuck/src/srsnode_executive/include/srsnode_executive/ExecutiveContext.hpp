/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/localization/map/MapStack.hpp>
#include <srslib_framework/robotics/Pose.hpp>
#include <srslib_framework/robotics/Velocity.hpp>

#include <srsnode_executive/resource/Resource.hpp>

namespace srs {

struct ExecutiveContext
{
    enum DirectionEnum {
        ENTERING, EXITING, STAYING
    };

    using ActiveLabelType = pair<LogicalMap::LabeledArea, DirectionEnum>;

    vector<ActiveLabelType> activeLabeledAreas;

    Velocity<> commandedVelocity;

    bool isRobotMoving;
    bool isRobotPaused;

    MapStack* mapStack;

    vector<ActiveLabelType> previousLabeledAreas;

    Resource resourceSound;
    Pose<> robotPose;
};

} // namespace srs
