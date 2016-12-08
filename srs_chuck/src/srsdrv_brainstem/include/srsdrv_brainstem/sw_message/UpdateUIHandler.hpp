/*
UpdateUIHandler.hpp * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/MsgUpdateUI.h>

#include <srslib_framework/platform/SoftwareMessageHandler.hpp>
#include <srslib_framework/ros/tap/subscriber/Subscriber.hpp>
#include <srslib_framework/ros/tap/subscriber/Observer.hpp>
#include <srslib_framework/ros/tap/TapBrainstemCmd_UpdateUI.hpp>

#include <srsdrv_brainstem/BrainStemMessages.h>

namespace srs {

class BrainStemMessageProcessor;

class UpdateUIHandler :
    public SoftwareMessageHandler<BrainStemMessageProcessor>,
    public Observer<Subscriber<srslib_framework::MsgUpdateUI>>
{
public:
    UpdateUIHandler(BrainStemMessageProcessor* owner);

    virtual ~UpdateUIHandler()
    {}

    void notified(Subscriber<srslib_framework::MsgUpdateUI>* subject);

private:
    HW_MESSAGE_BEGIN(UpdateUIData)
        uint8_t cmd;
        uint8_t entity;
        uint16_t mode;
    HW_MESSAGE_END

	TapBrainstemCmd_UpdateUI 					tapUpdateUI_;

	std::set<LED_ENTITIES>						setValidEntities_;

	std::map<LED_ENTITIES, std::set<LED_MODE>>	mapValidModes_;
};

} // namespace srs
