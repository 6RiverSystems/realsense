/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef STARGAZER_HPP_
#define STARGAZER_HPP_

#include <ros/ros.h>

namespace srs {

class Stargazer
{
public:
    Stargazer();

    ~Stargazer()
    {

    }

    void run();

private:
    constexpr static unsigned int REFRESH_RATE_HZ = 10;

    ros::NodeHandle rosNodeHandle_;
    ros::Publisher rosPubXXX_;
};

} // namespace srs

#endif  // STARGAZER_HPP_
