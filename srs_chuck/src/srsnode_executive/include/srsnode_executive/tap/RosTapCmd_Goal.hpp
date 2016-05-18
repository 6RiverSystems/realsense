/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef ROSTAPCMD_GOAL_HPP_
#define ROSTAPCMD_GOAL_HPP_

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
    typedef typename Pose<>::BaseType BaseType;

    RosTapCmd_Goal(ros::NodeHandle rosHandle) :
        RosTap(rosHandle, "Command 'Goal' Tap")
    {}

    ~RosTapCmd_Goal()
    {
        disconnectTap();
    }

    Pose<> getGoal()
    {
        setNewData(false);
        return currentGoal_;
    }

    void reset()
    {
        set(Time::time2number(ros::Time::now()), 0.0, 0.0, 0.0);
    }

    void set(double arrivalTime, BaseType x, BaseType y, BaseType theta)
    {
        currentGoal_ = Pose<>(arrivalTime, x, y, theta);
        setNewData(true);
    }

protected:
    bool connect()
    {
        rosSubscriber_ = rosNodeHandle_.subscribe("/cmd/goal", 10, &RosTapCmd_Goal::onGoal, this);
        return true;
    }

private:
    void onGoal(const geometry_msgs::PoseStampedConstPtr message)
    {
        set(Time::time2number(message->header.stamp),
            message->pose.position.x,
            message->pose.position.y,
            tf::getYaw(message->pose.orientation));
    }

    Pose<> currentGoal_;
};

} // namespace srs

#endif // ROSTAPCMD_GOAL_HPP_
