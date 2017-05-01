/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#include <BrainStemMessageProcessorInterface.hpp>

#include <sw_message/UpdateBodyLightsHandler.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
UpdateBodyLightsHandler::UpdateBodyLightsHandler(BrainStemMessageProcessorInterface* owner) :
	SoftwareMessage(owner)
{

}

////////////////////////////////////////////////////////////////////////////////////////////////////
void UpdateBodyLightsHandler::attach()
{
	tapUpdateBodyLights_.reset(new TapBrainstemCmd_UpdateBodyLights());

	tapUpdateBodyLights_->attach(this);
}

void UpdateBodyLightsHandler::notified(Subscriber<srslib_framework::MsgUpdateBodyLights>* subject)
{
	TapBrainstemCmd_UpdateBodyLights* tap = static_cast<TapBrainstemCmd_UpdateBodyLights*>(subject);

	srslib_framework::MsgUpdateBodyLights updateBodyLights = tap->pop();

	encodeData(updateBodyLights);
}

void UpdateBodyLightsHandler::encodeData(const srslib_framework::MsgUpdateBodyLights& updateBodyLights)
{
	for (auto uiElement : updateBodyLights.bodyLightUpdates)
	{
		BODY_LIGHTS_ENTITIES entity = static_cast<BODY_LIGHTS_ENTITIES>(uiElement.entity);

		// extract message data and store into an EntityState
		ENTITY_STATE entityState;
		entityState.lightCmd = static_cast<LED_COMMAND>(uiElement.lightCmd);
		entityState.startColor[0] = static_cast<uint8_t>( uiElement.startColor.r);
		entityState.startColor[1] = static_cast<uint8_t>( uiElement.startColor.g );
		entityState.startColor[2] = static_cast<uint8_t>( uiElement.startColor.b );
		entityState.endColor[0] = static_cast<uint8_t>( uiElement.endColor.r );
		entityState.endColor[1] = static_cast<uint8_t>( uiElement.endColor.g );
		entityState.endColor[2] = static_cast<uint8_t>( uiElement.endColor.b );
		entityState.frequency = static_cast<float>( uiElement.frequency );

		updateEntity(entity, entityState);
	}
}

void UpdateBodyLightsHandler::syncState()
{
	for (auto iter : mapEntityState_)
	{
		updateEntity(iter.first, iter.second);
	}
}

void UpdateBodyLightsHandler::updateEntity(const BODY_LIGHTS_ENTITIES& entity, const ENTITY_STATE& entityState)
{
	UpdateBodyLightsData msg = {
		static_cast<uint8_t>( BRAIN_STEM_CMD::SET_BODY_LIGHTS ),
		static_cast<uint8_t>( entity ),
		static_cast<uint8_t>( entityState.lightCmd ),
		static_cast<uint8_t>( entityState.startColor[0] ),
		static_cast<uint8_t>( entityState.startColor[1] ),
		static_cast<uint8_t>( entityState.startColor[2] ),
		static_cast<uint8_t>( entityState.endColor[0] ),
		static_cast<uint8_t>( entityState.endColor[1] ),
		static_cast<uint8_t>( entityState.endColor[2] ),
		static_cast<float>( entityState.frequency )
	};

	mapEntityState_[entity] = entityState;

	sendCommand(reinterpret_cast<char*>(&msg), sizeof(msg));
}
} // namespace srs
