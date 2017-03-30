/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <BrainStemMessages.hpp>
#include <sw_message/SoftwareMessage.hpp>

#include <srslib_framework/ros/tap/subscriber/Subscriber.hpp>
#include <srslib_framework/platform/observer/Observer.hpp>
#include <srslib_framework/ros/tap/TapBrainstemCmd_Ping.hpp>

namespace srs {

class BrainStemMessageProcessorInterface;

class PingHandler :
	public SoftwareMessage,
	public Observer<Subscriber<std_msgs::Bool>>
{
public:
    PingHandler(BrainStemMessageProcessorInterface* owner);

    virtual ~PingHandler() {}

    virtual void attach();

    virtual void notified(Subscriber<std_msgs::Bool>* subject);

    void encodeData(const bool& value);

private:

    std::shared_ptr<TapBrainstemCmd_Ping>	tapPing_;
};

} // namespace srs
