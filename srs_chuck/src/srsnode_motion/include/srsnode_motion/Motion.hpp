/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef MOTION_HPP_
#define MOTION_HPP_

#include <srslib_framework/ros/tap/RosTapJoy.hpp>

namespace srs {

class Motion
{
public:
    Motion();

    ~Motion()
    {
    }

    void run();

private:
    constexpr static unsigned int REFRESH_RATE_HZ = 20;

    ros::NodeHandle rosNodeHandle_;
};

} // namespace srs

#endif  // MOTION_HPP_
