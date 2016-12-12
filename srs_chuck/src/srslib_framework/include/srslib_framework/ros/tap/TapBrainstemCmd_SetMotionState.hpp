/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/ros/tap/subscriber/SubscriberSetMotionState.hpp>
#include <srslib_framework/ros/topics/ChuckTopics.hpp>

namespace srs {

class TapBrainstemCmd_SetMotionState :
    public SubscriberSetMotionState
{
public:
	TapBrainstemCmd_SetMotionState() :
        SubscriberSetMotionState(ChuckTopics::driver::BRAINSTEM_CMD_SET_MOTION_STATE)
    {}
};

} // namespace srs
