/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef TIME_HPP_
#define TIME_HPP_

#include <cmath>
#include <limits>
using namespace std;

#include <ros/ros.h>

namespace srs {

struct Time
{
    template<typename TYPE = double>
    inline static TYPE time2number(ros::Time& time)
    {
        return static_cast<TYPE>(time.nsec) * 1.0e-9 + static_cast<TYPE>(time.sec);
    }

    template<typename TYPE = double>
    inline static TYPE time2number(const ros::Time& time)
    {
        return static_cast<TYPE>(time.nsec) * 1.0e-9 + static_cast<TYPE>(time.sec);
    }
};

} // namespace srs

#endif // TIME_HPP_