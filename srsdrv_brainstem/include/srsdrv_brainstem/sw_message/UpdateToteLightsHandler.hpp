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
#include <srslib_framework/ros/tap/TapBrainstemCmd_UpdateToteLights.hpp>

namespace srs {

class BrainStemMessageProcessorInterface;

class UpdateToteLightsHandler :
	public SoftwareMessage,
	public Observer<Subscriber<srslib_framework::MsgUpdateToteLights>>
{
public:
	UpdateToteLightsHandler(BrainStemMessageProcessorInterface* owner);

	virtual ~UpdateToteLightsHandler() {}

	virtual void attach();

	void notified(Subscriber<srslib_framework::MsgUpdateToteLights>* subject);

	void encodeData(const srslib_framework::MsgUpdateToteLights& value);

	void syncState();

private:

	HW_MESSAGE_BEGIN(UpdateToteLightsData)
		uint8_t lengthStart;
		uint8_t widthStart;
		uint8_t levelStart;
		uint8_t lengthEnd;
		uint8_t widthEnd;
		uint8_t levelEnd;
		uint8_t lightCmd;
		uint8_t cmd;
		uint8_t startColorRed;
		uint8_t startColorGreen;
		uint8_t startColorBlue;
		uint8_t endColorRed;
		uint8_t endColorGreen;
		uint8_t endColorBlue;
		float frequency;
	HW_MESSAGE_END

	srslib_framework::MsgUpdateToteLights toteLightsState_;

	std::shared_ptr<TapBrainstemCmd_UpdateToteLights>	tapUpdateToteLights_;

};

} // namespace srs
