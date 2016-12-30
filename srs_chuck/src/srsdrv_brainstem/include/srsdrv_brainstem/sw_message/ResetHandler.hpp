/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <BrainStemMessages.hpp>

#include <srslib_framework/platform/SoftwareMessageHandler.hpp>
#include <srslib_framework/ros/tap/subscriber/Subscriber.hpp>
#include <srslib_framework/platform/observer/Observer.hpp>
#include <srslib_framework/ros/tap/TapBrainstemCmd_Reset.hpp>

namespace srs {

class BrainStemMessageProcessorInterface;

class ResetHandler :
    public SoftwareMessageHandler<BrainStemMessageProcessorInterface>,
    public Observer<Subscriber<std_msgs::Bool>>
{
public:
    ResetHandler(BrainStemMessageProcessorInterface* owner);

    virtual ~ResetHandler() {}

    virtual void attach();

    virtual void notified(Subscriber<std_msgs::Bool>* subject);

    void encodeData(const bool& value);

    void reset() { sentReset_ = false; };

private:


    HW_MESSAGE_BEGIN(WatchdogTimeoutData)
    	uint8_t cmd;
		char token[4];
    HW_MESSAGE_END

	bool sentReset_;

    std::shared_ptr<TapBrainstemCmd_Reset>	tapReset_;
};

} // namespace srs
