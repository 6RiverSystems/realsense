/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#include <BrainStemMessageProcessorInterface.hpp>

#include <sw_message/UpdateToteLightsHandler.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
UpdateToteLightsHandler::UpdateToteLightsHandler(BrainStemMessageProcessorInterface* owner) :
	SoftwareMessage(owner)
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void UpdateToteLightsHandler::attach()
{
	tapUpdateToteLights_.reset(new TapBrainstemCmd_UpdateToteLights());

	tapUpdateToteLights_->attach(this);
}

void UpdateToteLightsHandler::notified(Subscriber<srslib_framework::MsgUpdateToteLights>* subject)
{
	TapBrainstemCmd_UpdateToteLights* tap = static_cast<TapBrainstemCmd_UpdateToteLights*>(subject);

	srslib_framework::MsgUpdateToteLights updateToteLights = tap->pop();

	encodeData(updateToteLights);
}

void UpdateToteLightsHandler::encodeData(const srslib_framework::MsgUpdateToteLights& updateToteLights)
{
	toteLightsState_ = updateToteLights;

	UpdateToteLightsData msg = {
		static_cast<uint8_t>( BRAIN_STEM_CMD::SET_TOTE_LIGHTS ),
		static_cast<uint8_t>( updateToteLights.startSegment.x ),
		static_cast<uint8_t>( updateToteLights.startSegment.y ),
		static_cast<uint8_t>( updateToteLights.startSegment.z ),
		static_cast<uint8_t>( updateToteLights.endSegment.x ),
		static_cast<uint8_t>( updateToteLights.endSegment.y ),
		static_cast<uint8_t>( updateToteLights.endSegment.z ),
		static_cast<uint8_t>( updateToteLights.lightCmd ),
		static_cast<uint8_t>( updateToteLights.startColor.r ),
		static_cast<uint8_t>( updateToteLights.startColor.g ),
		static_cast<uint8_t>( updateToteLights.startColor.b ),
		static_cast<uint8_t>( updateToteLights.endColor.r ),
		static_cast<uint8_t>( updateToteLights.endColor.g ),
		static_cast<uint8_t>( updateToteLights.endColor.b ),
		static_cast<float>( updateToteLights.frequency )
	};

	sendCommand(reinterpret_cast<char*>(&msg), sizeof(msg));
}

void UpdateToteLightsHandler::syncState()
{
	encodeData(toteLightsState_);
}
} // namespace srs
