/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef ROSTRIGGERPAUSE_HPP_
#define ROSTRIGGERPAUSE_HPP_

#include <string>
using namespace std;

#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <srslib_framework/ros/service/RosServiceTrigger.hpp>

namespace srs {

class RosTriggerPause :
    public RosServiceTrigger
{
public:
    RosTriggerPause(ros::NodeHandle rosHandle) :
        RosServiceTrigger(rosHandle, "Trigger: Pause", "pause")
    {}
};

} // namespace srs

#endif // ROSTRIGGERPAUSE_HPP_
