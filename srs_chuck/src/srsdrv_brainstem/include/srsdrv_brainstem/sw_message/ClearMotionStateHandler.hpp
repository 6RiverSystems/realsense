/*
ClearMotionStatusHandler.hpp * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/MsgOperationalState.h>

#include <srslib_framework/platform/SoftwareMessageHandler.hpp>
#include <srslib_framework/ros/tap/subscriber/Subscriber.hpp>
#include <srslib_framework/ros/tap/subscriber/Observer.hpp>
#include <srslib_framework/ros/tap/TapBrainstemCmd_SetMotionState.hpp>

#include <srsdrv_brainstem/BrainStemMessages.h>

namespace srs {

class BrainStemMessageProcessor;

class ClearMotionStateHandler :
    public SoftwareMessageHandler<BrainStemMessageProcessor>,
    public Observer<Subscriber<srslib_framework::MsgOperationalState>>
{
public:
	ClearMotionStateHandler(BrainStemMessageProcessor* owner);

    virtual ~ClearMotionStateHandler()
    {}

    void notified(Subscriber<srslib_framework::MsgOperationalState>* subject);

    TapBrainstemCmd_SetMotionState	tapSetMotionState_;
};

} // namespace srs
