/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef ROSTAPGOAL_HPP_
#define ROSTAPGOAL_HPP_

#include <string>
using namespace std;

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>

#include <srslib_framework/math/Time.hpp>
#include <srslib_framework/robotics/Pose.hpp>
#include <srslib_framework/ros/RosTap.hpp>

namespace srs {

class RosTapCmd_Goal :
    public RosTap
{
public:
    RosTapCmd_Goal(string nodeName) :
        RosTap(nodeName, "Command 'Goal' Tap")
    {}

    ~RosTapCmd_Goal()
    {
        disconnectTap();
    }

    Pose<> getCurrentGoal()
    {
        setNewData(false);
        return currentGoal_;
    }

protected:
    bool connect()
    {
        rosSubscriber_ = rosNodeHandle_.subscribe("cmd/goal", 10, &RosTapCmd_Goal::onGoal, this);
        return true;
    }

private:
    void onGoal(const geometry_msgs::PoseStampedConstPtr message)
    {
        currentGoal_ = Pose<>(Time::time2number(message->header.stamp),
            message->pose.position.x,
            message->pose.position.y,
            tf::getYaw(message->pose.orientation));
        setNewData(true);
    }

    Pose<> currentGoal_;
};

} // namespace srs

#endif // ROSTAPGOAL_HPP_
