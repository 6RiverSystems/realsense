#include <srsdrv_brainstem/BrainStemMessageProcessor.h>

#include <ros/ros.h>
#include <sw_message/UpdateUIHandler.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
UpdateUIHandler::UpdateUIHandler(BrainStemMessageProcessor* owner) :
    SoftwareMessageHandler(owner)
{
	setValidEntities_.insert(LED_ENTITIES::TOTE0);
	setValidEntities_.insert(LED_ENTITIES::TOTE1);
	setValidEntities_.insert(LED_ENTITIES::TOTE2);
	setValidEntities_.insert(LED_ENTITIES::TOTE3);
	setValidEntities_.insert(LED_ENTITIES::TOTE4);
	setValidEntities_.insert(LED_ENTITIES::TOTE5);
	setValidEntities_.insert(LED_ENTITIES::TOTE6);
	setValidEntities_.insert(LED_ENTITIES::TOTE7);
	setValidEntities_.insert(LED_ENTITIES::ACTION);
	setValidEntities_.insert(LED_ENTITIES::PAUSE);
	setValidEntities_.insert(LED_ENTITIES::TAIL_LEFT);
	setValidEntities_.insert(LED_ENTITIES::TAIL_RIGHT);
	setValidEntities_.insert(LED_ENTITIES::SCANNER);

	std::set<LED_MODE> toteModes;
	toteModes.insert(LED_MODE::OFF);
	toteModes.insert(LED_MODE::GRAB);
	toteModes.insert(LED_MODE::PUT);

	mapValidModes_[LED_ENTITIES::TOTE0] = toteModes;
	mapValidModes_[LED_ENTITIES::TOTE1] = toteModes;
	mapValidModes_[LED_ENTITIES::TOTE2] = toteModes;
	mapValidModes_[LED_ENTITIES::TOTE3] = toteModes;
	mapValidModes_[LED_ENTITIES::TOTE4] = toteModes;
	mapValidModes_[LED_ENTITIES::TOTE5] = toteModes;
	mapValidModes_[LED_ENTITIES::TOTE6] = toteModes;
	mapValidModes_[LED_ENTITIES::TOTE7] = toteModes;

	mapValidModes_[LED_ENTITIES::ACTION] = toteModes;
	mapValidModes_[LED_ENTITIES::PAUSE] = toteModes;

	mapValidModes_[LED_ENTITIES::TAIL_LEFT] = toteModes;
	mapValidModes_[LED_ENTITIES::TAIL_RIGHT] = toteModes;

	mapValidModes_[LED_ENTITIES::SCANNER] = toteModes;

	tapUpdateUI_.attach(this);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void UpdateUIHandler::notified(Subscriber<srslib_framework::MsgUpdateUI>* subject)
{
	TapBrainstemCmd_UpdateUI* tap = static_cast<TapBrainstemCmd_UpdateUI*>(subject);

	srslib_framework::MsgUpdateUI updateUI = tap->pop();

	for (auto uiElement : updateUI.elements)
	{
		UpdateUIData msg = {
			static_cast<uint8_t>( BRAIN_STEM_CMD::UPDATE_LIGHT ),
			reinterpret_cast<uint8_t>( uiElement.element ),
			reinterpret_cast<uint8_t>( uiElement.mode )
		};

		getOwner()->sendCommand(reinterpret_cast<char*>(&msg), sizeof(msg));
	}
}

} // namespace srs
