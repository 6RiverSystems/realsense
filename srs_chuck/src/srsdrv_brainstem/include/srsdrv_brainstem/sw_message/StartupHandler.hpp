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
#include <srslib_framework/ros/tap/TapBrainstemCmd_Startup.hpp>

#include "../BrainStemMessages.hpp"

namespace srs {

class BrainStemMessageProcessorInterface;

class StartupHandler :
    public SoftwareMessageHandler<BrainStemMessageProcessorInterface>,
    public Observer<Subscriber<std_msgs::Bool>>
{
public:
    StartupHandler(BrainStemMessageProcessorInterface* owner);

    virtual ~StartupHandler() {}

    virtual void attach();

    void notified(Subscriber<std_msgs::Bool>* subject);

    void encodeData(const bool& value);

    std::shared_ptr<TapBrainstemCmd_Startup>	tapStartup_;
};

} // namespace srs
