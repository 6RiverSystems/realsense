/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <nav_msgs/Odometry.h>

#include <srslib_framework/localization/map/MapStack.hpp>
#include <srslib_framework/ros/tap/TapMapStack.hpp>
#include <srslib_framework/ros/tap/TapSensorOdometryPose.hpp>
#include <srslib_framework/ros/tap/subscriber/Observer.hpp>
#include <srslib_framework/ros/channel/ChannelBrainstemCmd_Sound.hpp>

namespace srs {

class LabeledAreasDetector
{
public:
    LabeledAreasDetector();

    virtual ~LabeledAreasDetector()
    {}

    void evaluatePose(Pose<> robotPose);

private:
    static const Sound SOUND_OFF;
    static const Sound WARNING_BEEP;

    enum DirectionEnum {
        ENTERING, EXITING
    };

    void interpretArea(LogicalMap::LabeledArea area, DirectionEnum direction);

    void updateMapStack();

    ChannelBrainstemCmd_Sound channelSound_;
    LogicalMap::LabeledAreaMapType currentLabeledAreas_;

    tf::TransformListener listener_;
    LogicalMap* logicalMap_;

    MapStack* srsMapStack_;

    TapMapStack tapMapStack_;
};

} // namespace srs
