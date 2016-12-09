/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
using namespace std;

#include <srsdrv_brainstem/BrainStemMessages.h>
#include <srsdrv_brainstem/hw_message/HardwareMessageHandler.hpp>

namespace srs {

class HardwareInfoHandler : public HardwareMessageHandler
{
public:
    static constexpr char HARDWARE_INFO_KEY = static_cast<char>(BRAIN_STEM_MSG::HARDWARE_INFO);

    static const string TOPIC_HARDWARE_INFO;

    HardwareInfoHandler();

    virtual ~HardwareInfoHandler()
    {}

    void receiveData(ros::Time currentTime, vector<char>& binaryData);

private:
    HW_MESSAGE_BEGIN(MsgHardwareInfo)
        uint8_t cmd;
        uint16_t uniqueId[8];
        uint8_t chassisGeneration;
        uint8_t brainstemHwVersion;
    HW_MESSAGE_END

    void publishHardwareInfo();

    unsigned int brainstemHwVersion_;
    string brainstemSwVersion_;

    unsigned int chassisGeneration_;

    ros::Publisher pubHardwareInfo_;

    string robotName_;

    string uid_;
};

} // namespace srs
