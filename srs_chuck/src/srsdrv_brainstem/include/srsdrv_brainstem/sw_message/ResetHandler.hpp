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
#include <srslib_framework/ros/tap/TapBrainstemCmd_Reset.hpp>

namespace srs {

class BrainStemMessageProcessorInterface;

class ResetHandler :
	public SoftwareMessage,
    public Observer<Subscriber<std_msgs::Bool>>
{
public:
    ResetHandler(BrainStemMessageProcessorInterface* owner);

    virtual ~ResetHandler() {}

    virtual void attach();

    virtual void notified(Subscriber<std_msgs::Bool>* subject);

    void encodeData(const bool& value);

private:

    HW_MESSAGE_BEGIN(WatchdogTimeoutData)
    	uint8_t cmd;
		char token[4];
    HW_MESSAGE_END

    std::shared_ptr<TapBrainstemCmd_Reset>	tapReset_;
};

} // namespace srs
