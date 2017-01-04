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
#include <srslib_framework/ros/tap/TapBrainstemCmd_UpdateUI.hpp>

namespace srs {

class BrainStemMessageProcessorInterface;

class UpdateUIHandler :
	public SoftwareMessage,
	public Observer<Subscriber<srslib_framework::MsgUpdateUI>>
{
public:
	UpdateUIHandler(BrainStemMessageProcessorInterface* owner);

	virtual ~UpdateUIHandler() {}

	virtual void attach();

	void notified(Subscriber<srslib_framework::MsgUpdateUI>* subject);

	void encodeData(const srslib_framework::MsgUpdateUI& value);

	void updateEntity(LED_ENTITIES entity, LED_MODE mode);

    void syncState();

private:

	HW_MESSAGE_BEGIN(UpdateUIData)
		uint8_t cmd;
		uint8_t entity;
		uint8_t mode;
	HW_MESSAGE_END

	std::map<LED_ENTITIES, LED_MODE>	setEntityMode_;

	std::shared_ptr<TapBrainstemCmd_UpdateUI>	tapUpdateUI_;

	std::set<LED_ENTITIES>						setValidEntities_;

	std::map<LED_ENTITIES, std::set<LED_MODE>>	mapValidModes_;
};

} // namespace srs
