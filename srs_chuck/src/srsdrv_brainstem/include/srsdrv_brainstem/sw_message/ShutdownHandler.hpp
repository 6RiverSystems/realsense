/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <std_msgs/Bool.h>

#include <srslib_framework/platform/SoftwareMessageHandler.hpp>
#include <srslib_framework/ros/tap/subscriber/Subscriber.hpp>
#include <srslib_framework/ros/tap/subscriber/Observer.hpp>
#include <srslib_framework/ros/tap/TapBrainstemCmd_Shutdown.hpp>

#include <srsdrv_brainstem/BrainStemMessages.h>

namespace srs {

class BrainStemMessageProcessor;

class ShutdownHandler :
    public SoftwareMessageHandler<BrainStemMessageProcessor>,
    public Observer<Subscriber<std_msgs::Bool>>
{
public:
    ShutdownHandler(BrainStemMessageProcessor* owner);

    virtual ~ShutdownHandler()
    {}

    void notified(Subscriber<std_msgs::Bool>* subject);

    TapBrainstemCmd_Shutdown	tapShutdown_;
};

} // namespace srs
