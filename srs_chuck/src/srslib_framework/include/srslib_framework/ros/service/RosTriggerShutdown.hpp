/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef ROSTRIGGERSHUTDOWN_HPP_
#define ROSTRIGGERSHUTDOWN_HPP_

#include <string>
using namespace std;

#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <srslib_framework/ros/service/RosServiceTrigger.hpp>

namespace srs {

class RosTriggerShutdown :
    public RosServiceTrigger
{
public:
    RosTriggerShutdown(ros::NodeHandle rosHandle) :
        RosServiceTrigger(rosHandle, "Trigger: Shutdown", "trg/shutdown")
    {}

    ~RosTriggerShutdown()
    {}

    bool isShutdownRequested()
    {
        return newRequestPending() && isTriggerRequested();
    }
};

} // namespace srs

#endif // ROSTRIGGERSHUTDOWN_HPP_
