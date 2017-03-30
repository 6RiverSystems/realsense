/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/localization/map/MapStack.hpp>
#include <srslib_framework/localization/map/mapnote/MapNotes.hpp>
#include <srslib_framework/localization/map/mapnote/NotePlaySound.hpp>
#include <srslib_framework/robotics/Pose.hpp>
#include <srslib_framework/robotics/Velocity.hpp>

#include <srsnode_executive/resource/Resource.hpp>

namespace srs {

struct GoalContext
{
    Pose<> goal;
    bool inQueue;
    std::string queue;
};

struct ExecutiveContext
{
    enum DirectionEnum {
        ENTERING, EXITING, STAYING
    };

    using ActiveLabelType = pair<LogicalMap::LabeledArea, DirectionEnum>;

    vector<ActiveLabelType> activeLabeledAreas;

    Velocity<> commandedVelocity;

    GoalContext goal;

    bool isRobotMoving;
    bool isRobotPaused;

    MapStack* mapStack;
    float maxVelocity;

    vector<ActiveLabelType> previousLabeledAreas;

    Resource resourceSound;
    Pose<> robotPose;
};

} // namespace srs
