/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
using namespace std;

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include <srslib_framework/robotics/Pose.hpp>
#include <srslib_framework/robotics/Velocity.hpp>
#include <srslib_framework/ros/tap/subscriber/SubscriberSingleData.hpp>
#include <srslib_framework/ros/message/PoseMessageFactory.hpp>

namespace srs {

class SubscriberRosOdometry :
    public SubscriberSingleData<nav_msgs::Odometry, Pose<>>
{
public:
    SubscriberRosOdometry(string topic,
        unsigned int queueLength = 10,
        string nameSpace = "~") :
            SubscriberSingleData(topic, queueLength, nameSpace)
    {
        reset();
    }

    ~SubscriberRosOdometry()
    {}

    Velocity<> peekVelocity() const
    {
        return velocity_;
    }

    Velocity<> popVelocity()
    {
        Subscriber::declareStale();
        return velocity_;
    }

    void receiveData(const nav_msgs::Odometry::ConstPtr message)
    {
        set(PoseMessageFactory::msg2Pose(message));
    }

    void reset()
    {
        // Reset the data, and then reset the subscriber
        set(Pose<>::INVALID);
        velocity_ = Velocity<>::INVALID;

        Subscriber::reset();
    }

private:
    Velocity<> velocity_;
};

} // namespace srs
