/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef JOYSTICKADAPTER_HPP_
#define JOYSTICKADAPTER_HPP_

#include <string>
using namespace std;

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
    constexpr static unsigned int REFRESH_RATE_HZ = 5;

    constexpr static double RATIO_LINEAR = 0.7;
    constexpr static double RATIO_ANGULAR = 0.3;
    constexpr static double THRESHOLD_LINEAR = 0.2;
    constexpr static double THRESHOLD_ANGULAR = 0.05;

    void evaluateTriggers();

    bool joystickLatched_;

    ros::Publisher pubCommand_;
    ros::Publisher pubJoystickLatched_;

    ros::NodeHandle rosNodeHandle_;

    RosTapJoy<> tapJoy_;
    RosTriggerShutdown triggerShutdown_;
};

} // namespace srs

#endif  // JOYSTICKADAPTER_HPP_
