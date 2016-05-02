/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef REFLEXES_HPP_
#define REFLEXES_HPP_

#include <srslib_framework/ros/tap/RosTapJoy.hpp>

namespace srs {

class Reflexes
{
public:
    Reflexes();

    ~Reflexes()
    {
    }

    void run();

private:
    constexpr static unsigned int REFRESH_RATE_HZ = 200;

    ros::NodeHandle rosNodeHandle_;
};

} // namespace srs

#endif  // REFLEXES_HPP_
