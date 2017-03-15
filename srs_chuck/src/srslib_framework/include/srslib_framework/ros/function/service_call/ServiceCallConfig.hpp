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
#include <dynamic_reconfigure/IntParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

namespace srs {

struct ServiceConfig
{
    static bool call(std::string node, dynamic_reconfigure::Config configuration)
    {
        ros::NodeHandle rosNodeHandle;

        string fullServiceName = node + "/set_parameters";

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

template<typename TYPE>
class ServiceCallConfig
{
    static bool set(string node, string parameter, TYPE value)
    {
        return false;
    }
};

template<>
struct ServiceCallConfig<int>
{
    static bool set(string node, string parameter, int value)
    {
        dynamic_reconfigure::IntParameter intParameter;
        intParameter.name = parameter;
        intParameter.value = value;

        dynamic_reconfigure::Config configuration;
        configuration.ints.push_back(intParameter);

        return ServiceConfig::call(node, configuration);
    }
};

template<>
struct ServiceCallConfig<double>
{
    static bool set(string node, string parameter, double value)
    {
        dynamic_reconfigure::DoubleParameter doubleParameter;
        doubleParameter.name = parameter;
        doubleParameter.value = value;

        dynamic_reconfigure::Config configuration;
        configuration.doubles.push_back(doubleParameter);

        return ServiceConfig::call(node, configuration);
    }
};

} // namespace srs
