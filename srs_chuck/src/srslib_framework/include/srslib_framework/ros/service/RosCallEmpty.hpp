/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef ROSCALLEMPTY_HPP_
#define ROSCALLEMPTY_HPP_

#include <string>
using namespace std;

#include <ros/ros.h>
#include <std_srvs/Empty.h>

namespace srs {

struct RosCallEmpty
{
public:
    static bool call(string node, string trigger)
    {
        ros::NodeHandle rosNodeHandle;

        string fullServiceName = node + trigger;

        ros::ServiceClient client = rosNodeHandle.serviceClient<std_srvs::Empty>(fullServiceName);
        std_srvs::Empty::Request req;
        std_srvs::Empty::Response resp;

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

#endif // ROSCALLEMPTY_HPP_
