/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef JOYSTICKADAPTER_HPP_
#define JOYSTICKADAPTER_HPP_

#include <srslib_framework/ros/tap/RosTapJoy.hpp>

namespace srs {

class JoystickAdapter
{
public:
    JoystickAdapter();

    ~JoystickAdapter()
    {
        tapJoy_.disconnectTap();
    }

    void run();

private:
    constexpr static unsigned int REFRESH_RATE_HZ = 10;

    RosTapJoy<> tapJoy_;

    ros::NodeHandle rosNodeHandle_;
    ros::Publisher rosPubCmdVel_;
};

} // namespace srs

#endif  // JOYSTICKADAPTER_HPP_
