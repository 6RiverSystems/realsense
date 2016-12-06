/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
using namespace std;

#include <ros/ros.h>
#include <geometry_msgs/PolygonStamped.h>

#include <srslib_framework/robotics/Pose.hpp>
#include <srslib_framework/ros/tap/subscriber/SubscriberSingleData.hpp>
#include <srslib_framework/ros/message/PolygonMessageFactory.hpp>

namespace srs {

class SubscriberPolygonStamped :
    public SubscriberSingleData<geometry_msgs::PolygonStamped, std::vector<Pose<>>>
{
public:
    SubscriberPolygonStamped(string topic,
        unsigned int queueLength = 10,
        string nameSpace = "~") :
            SubscriberSingleData(topic, queueLength, nameSpace)
    {
        reset();
    }

    ~SubscriberPolygonStamped()
    {}

    void receiveData(const geometry_msgs::PolygonStamped::ConstPtr message)
    {
        set(PoseMessageFactory::polygonStamped2Poses(message));
    }

    void reset()
    {
        // Reset the data, and then reset the subscriber
        set(std::vector<Pose<>>());

        Subscriber::reset();
    }
};

} // namespace srs
