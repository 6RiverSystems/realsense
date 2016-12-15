/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
using namespace std;

#include <ros/ros.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

namespace srs {

template<typename TYPE>
struct ServiceCallConfig
{
    static bool call(string node, string parameter, TYPE value)
    {
        return false;
    }
};

template<>
struct ServiceCallConfig<double>
{
    static bool call(string node, string parameter, double value)
    {
        ros::NodeHandle rosNodeHandle;

        string fullServiceName = node + "/set_parameters";

        dynamic_reconfigure::Config configuration;

        dynamic_reconfigure::DoubleParameter doubleParameter;
        doubleParameter.name = parameter;
        doubleParameter.value = value;
        configuration.doubles.push_back(doubleParameter);

        dynamic_reconfigure::ReconfigureRequest request;
        request.config = configuration;

        dynamic_reconfigure::ReconfigureResponse response;

        if (ros::service::call(fullServiceName, request, response))
        {
            ROS_DEBUG_STREAM_NAMED("ros_service_call", "The node " << node <<
                " responded to a set_parameters request.");

            return true;
        }

        ROS_DEBUG_STREAM_NAMED("ros_service_call", "The node " << node <<
            " did not respond to a set_parameters request.");

        return false;
    }
};

} // namespace srs
