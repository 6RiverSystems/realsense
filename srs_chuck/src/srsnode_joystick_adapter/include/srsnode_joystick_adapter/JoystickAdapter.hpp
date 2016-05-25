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
using namespace srsnode_joystick_adapter;

#include <dynamic_reconfigure/server.h>

#include <srslib_framework/ros/service/RosTriggerShutdown.hpp>
#include <srslib_framework/ros/tap/RosTapJoy.hpp>

namespace srs {

class JoystickAdapter
{
public:
    JoystickAdapter(string nodeName);

    ~JoystickAdapter()
    {
        tapJoy_.disconnectTap();
    }

    void run();

private:
    constexpr static unsigned int REFRESH_RATE_HZ = 10;

    void evaluateTriggers();

    void onConfigChange(JoystickConfig& config, uint32_t level);

    JoystickConfig configuration_;
    dynamic_reconfigure::Server<JoystickConfig> configServer_;

    bool joystickLatched_;

    ros::Publisher pubCommand_;
    ros::Publisher pubJoystickLatched_;

    ros::NodeHandle rosNodeHandle_;

    RosTapJoy<> tapJoy_;
    RosTriggerShutdown triggerShutdown_;
};

} // namespace srs

#endif  // JOYSTICKADAPTER_HPP_
