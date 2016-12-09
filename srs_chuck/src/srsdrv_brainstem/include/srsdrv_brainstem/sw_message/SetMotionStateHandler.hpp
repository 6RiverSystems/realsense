/*
SetMotionStateHandler.hpp * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/MsgSetOperationalState.h>

#include <srslib_framework/platform/SoftwareMessageHandler.hpp>
#include <srslib_framework/ros/tap/subscriber/Subscriber.hpp>
#include <srslib_framework/ros/tap/subscriber/Observer.hpp>
#include <srslib_framework/ros/tap/TapBrainstemCmd_SetMotionState.hpp>

#include <srsdrv_brainstem/BrainStemMessages.h>

namespace srs {

class BrainStemMessageProcessor;

class SetMotionStateHandler :
    public SoftwareMessageHandler<BrainStemMessageProcessor>,
    public Observer<Subscriber<srslib_framework::MsgSetOperationalState>>
{
public:
    SetMotionStateHandler(BrainStemMessageProcessor* owner);

    virtual ~SetMotionStateHandler()
    {}

    void notified(Subscriber<srslib_framework::MsgSetOperationalState>* subject);

    TapBrainstemCmd_SetMotionState	tapSetMotionState_;
};

} // namespace srs
