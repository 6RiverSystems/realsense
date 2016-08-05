/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef MIDBRAIN_HPP_
#define MIDBRAIN_HPP_

#include <ros/ros.h>

#include <srslib_framework/ros/service/RosTriggerShutdown.hpp>
#include <srslib_framework/ros/service/RosTriggerPause.hpp>
#include <srsnode_midbrain/Reflexes.hpp>

namespace srs {

class Midbrain
{
public:
    Midbrain(string nodeName);

    ~Midbrain()
    {}

    void run();

private:
    constexpr static double REFRESH_RATE_HZ = 200;

    void evaluateTriggers();

    void createSubscribers();

    void createPublishers();

    ros::NodeHandle rosNodeHandle_;

    RosTriggerShutdown triggerShutdown_;
    RosTriggerPause triggerPause_;
    Reflexes reflexes_;
};

} // namespace srs

#endif  // MIDBRAIN_HPP_