/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef JOYSTICKADAPTER_HPP_
#define JOYSTICKADAPTER_HPP_

#include <string>
using namespace std;

#include <srsnode_joystick_adapter/JoystickConfig.h>

#include <dynamic_reconfigure/server.h>

#include <srslib_framework/robotics/device/JoypadState.hpp>
#include <srslib_framework/ros/channel/ChannelJoypad.hpp>
#include <srslib_framework/ros/service/RosTriggerShutdown.hpp>
#include <srslib_framework/ros/tap/TapJoy.hpp>

namespace srs {

class JoystickAdapter
{
public:
    JoystickAdapter(string nodeName);

    ~JoystickAdapter()
    {}

    void run();

private:
    constexpr static double REFRESH_RATE_HZ = 20;

    void evaluateTriggers();

    void onConfigChange(srsnode_joystick_adapter::JoystickConfig& config, uint32_t level);

    srsnode_joystick_adapter::JoystickConfig configuration_;
    dynamic_reconfigure::Server<srsnode_joystick_adapter::JoystickConfig> configServer_;

    JoypadState joypadState_;

    Velocity<> currentLeftVelocity_;
    Velocity<> currentRightVelocity_;

    ChannelJoypad channelJoypad_;

    ros::NodeHandle rosNodeHandle_;

    TapJoy tapJoy_;
    RosTriggerShutdown triggerShutdown_;
};

} // namespace srs

#endif  // JOYSTICKADAPTER_HPP_
