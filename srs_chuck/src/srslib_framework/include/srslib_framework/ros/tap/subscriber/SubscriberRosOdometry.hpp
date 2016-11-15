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
#include <srslib_framework/ros/tap/subscriber/RosSubscriberSingleData.hpp>
#include <srslib_framework/ros/message/PoseMessageFactory.hpp>

namespace srs {

class SubscriberRosOdometry :
    public RosSubscriberSingleData<nav_msgs::Odometry, Pose<>>
{
public:
    SubscriberRosOdometry(string topic,
        unsigned int queueLength = 10,
        string nameSpace = "~") :
            RosSubscriberSingleData(topic, queueLength, nameSpace)
    {
        reset();
    }

    ~SubscriberRosOdometry()
    {}

    double peekAngularVelocity() const
    {
        return angularVelocity_;
    }

    double peekLinearVelocity() const
    {
        return linearVelocity_;
    }

    double popAngularVelocity()
    {
        RosSubscriber::declareStale();
        return angularVelocity_;
    }

    double popLinearVelocity()
    {
        RosSubscriber::declareStale();
        return linearVelocity_;
    }

    void receiveData(const nav_msgs::Odometry::ConstPtr message)
    {
        set(PoseMessageFactory::msg2Pose(message));
    }

    void reset()
    {
        // Reset the data, and then reset the subscriber
        set(Pose<>::INVALID);
        set(Pose<>::INVALID);

        linearVelocity_ = 0.0;
        angularVelocity_ = 0.0;

        RosSubscriber::reset();
    }

private:
    double linearVelocity_;
    double angularVelocity_;
};

} // namespace srs
