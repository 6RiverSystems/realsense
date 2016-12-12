/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/OdometryRpm.h>

#include <srslib_framework/platform/SoftwareMessageHandler.hpp>
#include <srslib_framework/ros/tap/subscriber/Subscriber.hpp>
#include <srslib_framework/ros/tap/subscriber/Observer.hpp>
#include <srslib_framework/ros/tap/TapBrainstemCmd_OdometryRpm.hpp>

#include "../BrainStemMessages.hpp"

namespace srs {

class BrainStemMessageProcessorInterface;

class SetOdometryRpmHandler :
    public SoftwareMessageHandler<BrainStemMessageProcessorInterface>,
    public Observer<Subscriber<srslib_framework::OdometryRpm>>
{
public:
    SetOdometryRpmHandler(BrainStemMessageProcessorInterface* owner);

    virtual ~SetOdometryRpmHandler() {}

    virtual void attach();

    void notified(Subscriber<srslib_framework::OdometryRpm>* subject);

    void encodeData(const srslib_framework::OdometryRpm& value);

private:
    HW_MESSAGE_BEGIN(RawOdometryData)
		uint8_t cmd;
		float rpm_left_wheel;
		float rpm_right_wheel;
	HW_MESSAGE_END

	std::shared_ptr<TapBrainstemCmd_OdometryRpm>	tapOdometryRpm_;
};

} // namespace srs
