/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <BrainStemMessages.hpp>
#include <hw_message/HardwareMessageHandler.hpp>

#include <srslib_framework/ros/channel/ChannelBrainstemHardwareInfo.hpp>

namespace srs {

class HardwareInfoHandler : public HardwareMessageHandler
{
public:

    HardwareInfoHandler(ChannelBrainstemHardwareInfo::Interface& publisher);

    virtual ~HardwareInfoHandler() {}

    void receiveMessage(ros::Time currentTime, HardwareMessage& msg);

    bool hasValidMessage() const { return hasValidMessage_; };

private:

    HW_MESSAGE_BEGIN(HardwareInfoData)
        uint8_t cmd;
        uint16_t uniqueId[8];
        uint8_t chassisGeneration;
        uint8_t brainstemHwVersion;
        // char brainstemFirmwareVersion* (null terminated string)
    HW_MESSAGE_END

    HW_MESSAGE_BEGIN(HardwareInfoData2)
		uint8_t cmd;
		uint16_t uniqueId[8];
		uint8_t chassisGeneration;
		uint8_t brainstemHwVersion;
        // char brainstemFirmwareVersion* (null terminated string)
        // uint8_t numberOfBatteries;
    	// MsgBatteryInfo[numberOfBatteries];
    HW_MESSAGE_END

    HW_MESSAGE_BEGIN(BatteryInfoData)
    	char manufacturer[13];
    	char serialNumber[33];
    HW_MESSAGE_END

	bool hasValidMessage_;

    srslib_framework::MsgHardwareInfo hardwareInfoMsg_;

    ChannelBrainstemHardwareInfo::Interface& publisher_;
};

} // namespace srs
