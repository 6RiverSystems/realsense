/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
#include <functional>

using namespace std;

#include <srsdrv_brainstem/BrainStemMessages.h>
#include <srsdrv_brainstem/hw_message/HardwareMessageHandler.hpp>
#include <srslib_framework/MsgHardwareInfo.h>

namespace srs {

class HardwareInfoHandler : public HardwareMessageHandler
{
public:

	typedef std::function<void(srslib_framework::MsgHardwareInfo&)> HardwareInfoFn;

    HardwareInfoHandler(HardwareInfoFn hardareInfoCallback = [&](srslib_framework::MsgHardwareInfo&) {});

    virtual ~HardwareInfoHandler() {}

    bool receiveMessage(ros::Time currentTime, HardwareMessage& msg);

private:

    HW_MESSAGE_BEGIN(HardwareInfoMsg1)
        uint8_t cmd;
        uint16_t uniqueId[8];
        uint8_t chassisGeneration;
        uint8_t brainstemHwVersion;
        //char brainstemFirmwareVersion* (null terminated string)
    HW_MESSAGE_END

    HW_MESSAGE_BEGIN(MsgHardwareInfo2)
		uint8_t cmd;
		uint16_t uniqueId[8];
		uint8_t chassisGeneration;
		uint8_t brainstemHwVersion;
        char brainstemFirmwareVersion[64];
        uint8_t numberOfBatteries;
    	// MsgBatteryInfo[numberOfBatteries];
    HW_MESSAGE_END

    HW_MESSAGE_BEGIN(MsgBatteryInfo)
    	char manufacturer[13];
    	char serialNumber[33];
    HW_MESSAGE_END

    srslib_framework::MsgHardwareInfo hardwareInfoMsg_;

    HardwareInfoFn hardwareInfoCallback_;
};

} // namespace srs
