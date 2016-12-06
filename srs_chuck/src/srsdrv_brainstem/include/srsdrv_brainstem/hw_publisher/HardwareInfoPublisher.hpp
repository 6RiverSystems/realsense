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
#include <srslib_framework/MsgHardwareInfo.h>

namespace srs {

class HardwareInfoPublisher
{
public:

    HardwareInfoPublisher(string nameSpace = "~");

    virtual ~HardwareInfoPublisher()
    {}

    void publishHardwareInfo(srslib_framework::MsgHardwareInfo& hardwareInfoMsg);

private:

    ros::NodeHandle rosNodeHandle_;

    ros::Publisher publisher_;
};

} // namespace srs
