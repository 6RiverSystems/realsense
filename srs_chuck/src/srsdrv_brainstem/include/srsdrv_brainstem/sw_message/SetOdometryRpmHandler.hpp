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

#include <srsdrv_brainstem/BrainStemMessages.h>

namespace srs {

class BrainStemMessageProcessor;

class SetOdometryRpmHandler :
    public SoftwareMessageHandler<BrainStemMessageProcessor>,
    public Observer<Subscriber<srslib_framework::OdometryRpm>>
{
public:
    SetOdometryRpmHandler(BrainStemMessageProcessor* owner);

    virtual ~SetOdometryRpmHandler()
    {}

    void notified(Subscriber<srslib_framework::OdometryRpm>* subject);

private:
    HW_MESSAGE_BEGIN(RawOdometryData)
		uint8_t cmd;
		float rpm_left_wheel;
		float rpm_right_wheel;
	HW_MESSAGE_END

	TapBrainstemCmd_OdometryRpm	tapOdometryRpm_;
};

} // namespace srs
