/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
using namespace std;

#include <ros/ros.h>
#include <std_srvs/SetBool.h>

namespace srs {

struct ServiceCallSetBool
{
public:
    static bool call(string node, string serviceName, bool value)
    {
        ros::NodeHandle rosNodeHandle;

        string fullServiceName = node + serviceName;

        std_srvs::SetBool::Request request;
        std_srvs::SetBool::Response response;

        req.data = value;
        if (ros::service::call(fullServiceName, request, response))
        {
            ROS_DEBUG_STREAM_NAMED("ros_service_call", "The node " << node <<
                " responded to a " << serviceName << " request.");

            return true;
        }

        ROS_DEBUG_STREAM_NAMED("ros_service_call", "The node " << node <<
            " did not respond to a " << serviceName << " request.");

        return false;
    }
};

} // namespace srs
