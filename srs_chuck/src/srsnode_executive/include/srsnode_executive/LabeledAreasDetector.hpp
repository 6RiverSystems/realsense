/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <nav_msgs/Odometry.h>

#include <srslib_framework/localization/map/MapStack.hpp>
#include <srslib_framework/ros/tap/TapMapStack.hpp>

#include <srsnode_executive/request/RequestWarningSound.hpp>

namespace srs {

class LabeledAreasDetector
{
public:
    LabeledAreasDetector();

    virtual ~LabeledAreasDetector()
    {}

    void evaluatePose(Pose<> robotPose);

private:
    enum DirectionEnum {
        ENTERING, EXITING, STAYING
    };

    void executeRequests();

    void interpretArea(LogicalMap::LabeledArea area, DirectionEnum direction);

    void updateMapStack();

    LogicalMap::LabeledAreaMapType currentLabeledAreas_;

    tf::TransformListener listener_;
    LogicalMap* logicalMap_;

    RequestWarningSound requestWarningSound_;

    MapStack* srsMapStack_;

    TapMapStack tapMapStack_;
};

} // namespace srs
