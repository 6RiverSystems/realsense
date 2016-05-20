/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef REFLEXES_HPP_
#define REFLEXES_HPP_

#include <ros/ros.h>

#include <srslib_framework/ros/service/RosTriggerShutdown.hpp>

namespace srs {

class Reflexes
{
public:
    Reflexes(string nodeName);

    ~Reflexes()
    {}

    void run();

private:
    constexpr static unsigned int REFRESH_RATE_HZ = 200;

    void evaluateTriggers();

    ros::NodeHandle rosNodeHandle_;

    RosTriggerShutdown triggerShutdown_;
};

} // namespace srs

#endif  // REFLEXES_HPP_
