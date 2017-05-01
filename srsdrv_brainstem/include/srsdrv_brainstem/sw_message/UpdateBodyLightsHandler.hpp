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
#include <srslib_framework/ros/tap/TapBrainstemCmd_UpdateBodyLights.hpp>

namespace srs {

class BrainStemMessageProcessorInterface;

class UpdateBodyLightsHandler :
	public SoftwareMessage,
	public Observer<Subscriber<srslib_framework::MsgUpdateBodyLights>>
{

public:

	UpdateBodyLightsHandler(BrainStemMessageProcessorInterface* owner);

	virtual ~UpdateBodyLightsHandler() {}

	virtual void attach();

	void notified(Subscriber<srslib_framework::MsgUpdateBodyLights>* subject);

	void encodeData(const srslib_framework::MsgUpdateBodyLights& value);

	void updateEntity(const BODY_LIGHTS_ENTITIES& entity, const ENTITY_STATE& entityState);

	void syncState();

private:

	HW_MESSAGE_BEGIN(UpdateBodyLightsData)
		uint8_t cmd;
		uint8_t entity;
		uint8_t lightCmd;
		uint8_t startColorRed;
		uint8_t startColorGreen;
		uint8_t startColorBlue;
		uint8_t endColorRed;
		uint8_t endColorGreen;
		uint8_t endColorBlue;
		float frequency;
	HW_MESSAGE_END

	std::map<BODY_LIGHTS_ENTITIES, ENTITY_STATE>	mapEntityState_;

	std::shared_ptr<TapBrainstemCmd_UpdateBodyLights>	tapUpdateBodyLights_;

};

} // namespace srs