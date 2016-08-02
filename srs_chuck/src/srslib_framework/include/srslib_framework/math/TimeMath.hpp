/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef TIMEMATH_HPP_
#define TIMEMATH_HPP_

#include <cmath>
#include <limits>
using namespace std;

#include <ros/ros.h>

namespace srs {

struct TimeMath
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

    template<typename TYPE = double>
    inline static bool isTimeElapsed(TYPE difference,
        const ros::Time& t1, const ros::Time& t2 = ros::Time::now())
    {
        return (t2.toSec() - t1.toSec()) > difference;
    }
};

} // namespace srs

#endif // TIMEMATH_HPP_
