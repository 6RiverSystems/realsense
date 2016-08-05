/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef ROSCALLSETBOOL_HPP_
#define ROSCALLSETBOOL_HPP_

#include <string>
using namespace std;

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>

namespace srs {

struct RosCallSetBool
{
public:
    static bool call(string node, string trigger, bool value)
    {
        ros::NodeHandle rosNodeHandle;

        string fullServiceName = node + trigger;

        ros::ServiceClient client = rosNodeHandle.serviceClient<std_srvs::SetBool>(fullServiceName);
        std_srvs::SetBool::Request req;
        std_srvs::SetBool::Response resp;

        req.data = value;
        if (client.call(req, resp))
        {
            ROS_DEBUG_STREAM_NAMED("ros_service_call", "The node " << node <<
                " responded to a " << trigger << " request.");

            return true;
        }

        ROS_DEBUG_STREAM_NAMED("ros_service_call", "The node " << node <<
            " did not respond to a " << trigger << " request.");

        return false;
    }
};

} // namespace srs

#endif // ROSCALLSETBOOL_HPP_