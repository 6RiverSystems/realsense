/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef ROSCALLSOLUTION_HPP_
#define ROSCALLSOLUTION_HPP_

#include <string>
using namespace std;

#include <ros/ros.h>
#include <srslib_framework/ExecuteSolution.h>

#include <srslib_framework/ros/message/SolutionMessageFactory.hpp>

namespace srs {

struct RosCallSolution
{
public:
    static bool call(string node, string trigger, Solution<Grid2dSolutionItem>* value)
    {
        ros::NodeHandle rosNodeHandle;

        string fullServiceName = node + trigger;

        ros::ServiceClient client = rosNodeHandle.serviceClient<srslib_framework::ExecuteSolution>
            (fullServiceName);
        srslib_framework::ExecuteSolution::Request req;
        srslib_framework::ExecuteSolution::Response resp;

        req.solution = SolutionMessageFactory::gridSolution2Msg(*value);
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

#endif // ROSCALLSOLUTION_HPP_
