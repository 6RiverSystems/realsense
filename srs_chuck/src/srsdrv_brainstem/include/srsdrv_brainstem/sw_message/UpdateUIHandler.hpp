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
#include <srslib_framework/ros/tap/TapBrainstemCmd_UpdateUI.hpp>

namespace srs {

class BrainStemMessageProcessorInterface;

class UpdateUIHandler :
	public SoftwareMessageHandler<BrainStemMessageProcessorInterface>,
	public Observer<Subscriber<srslib_framework::MsgUpdateUI>>
{
public:
	UpdateUIHandler(BrainStemMessageProcessorInterface* owner);

	virtual ~UpdateUIHandler() {}

	virtual void attach();

	void notified(Subscriber<srslib_framework::MsgUpdateUI>* subject);

	void encodeData(const srslib_framework::MsgUpdateUI& value);

    void syncState();

private:

	HW_MESSAGE_BEGIN(UpdateUIData)
		uint8_t cmd;
		uint8_t entity;
		uint8_t mode;
	HW_MESSAGE_END

	srslib_framework::MsgUpdateUI updateUI_;

	std::shared_ptr<TapBrainstemCmd_UpdateUI>	tapUpdateUI_;

	std::set<LED_ENTITIES>						setValidEntities_;

	std::map<LED_ENTITIES, std::set<LED_MODE>>	mapValidModes_;
};

} // namespace srs
