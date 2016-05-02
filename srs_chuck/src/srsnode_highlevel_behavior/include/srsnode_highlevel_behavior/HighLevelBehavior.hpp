/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef HIGHLEVELBEHAVIOR_HPP_
#define HIGHLEVELBEHAVIOR_HPP_

#include <srslib_framework/ros/tap/RosTapJoy.hpp>

namespace srs {

class HighLevelBehavior
{
public:
    HighLevelBehavior();

    ~HighLevelBehavior()
    {
    }

    void run();

private:
    constexpr static unsigned int REFRESH_RATE_HZ = 10;

    ros::NodeHandle rosNodeHandle_;
};

} // namespace srs

#endif  // HIGHLEVELBEHAVIOR_HPP_
