/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef ROSTAPYOUAREHERE_HPP_
#define ROSTAPYOUAREHERE_HPP_

#include <string>
using namespace std;

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <srslib_framework/math/Time.hpp>
#include <srslib_framework/robotics/Pose.hpp>
#include <srslib_framework/ros/RosTap.hpp>

namespace srs {

class RosTapCmd_YouAreHere :
    public RosTap
{
public:
    typedef typename Pose<>::BaseType BaseType;

    RosTapCmd_YouAreHere(string nodeName) :
        RosTap(nodeName, "Command 'You Are Here' Tap")
    {}

    ~RosTapCmd_YouAreHere()
    {
        disconnectTap();
    }

    Pose<> getRobotPose()
    {
        setNewData(false);
        return robotPose_;
    }

    void reset()
    {
        set(Time::time2number(ros::Time::now()), 2.0, 2.0, 0.0);
    }

    void set(double arrivalTime, BaseType x, BaseType y, BaseType theta)
    {
        robotPose_ = Pose<>(arrivalTime, x, y, theta);
        setNewData(true);
    }

protected:
    bool connect()
    {
        rosSubscriber_ = rosNodeHandle_.subscribe("cmd/you_are_here", 10,
            &RosTapCmd_YouAreHere::onYouAreHere, this);
        return true;
    }

private:
    void onYouAreHere(geometry_msgs::PoseWithCovarianceStampedConstPtr message)
    {
        set(Time::time2number(message->header.stamp),
            message->pose.pose.position.x,
            message->pose.pose.position.y,
            tf::getYaw(message->pose.pose.orientation));
    }

    Pose<> robotPose_;
};

} // namespace srs

#endif // ROSTAPYOUAREHERE_HPP_