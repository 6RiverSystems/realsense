/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef ROSTAPCMDVELOCITY_HPP_
#define ROSTAPCMDVELOCITY_HPP_

#include <string>
using namespace std;

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <srslib_framework/ros/RosTap.hpp>
#include <srslib_framework/robotics/Velocity.hpp>

namespace srs {

class RosTapCmdVelocity :
    public RosTap
{
public:
    typedef typename Velocity<>::BaseType BaseType;

    RosTapCmdVelocity() :
        RosTap("/internal/drivers/brainstem/cmd_velocity", "CmdVelocity Tap"),
        currentCmdVelocity_()
    {}

    ~RosTapCmdVelocity()
    {
        disconnectTap();
    }

    Velocity<> getVelocity()
    {
        setNewData(false);
        return currentCmdVelocity_;
    }

    void reset()
    {
        RosTap::reset();

        set(TimeMath::time2number(ros::Time::now()), 0.0, 0.0);
    }

    void set(double arrivalTime, BaseType linear, BaseType angular)
    {
        currentCmdVelocity_ = Velocity<>(linear, angular);
        setNewData(true);
    }

protected:
    bool connect()
    {
        rosSubscriber_ = rosNodeHandle_.subscribe(getTopic(), 1000, &RosTapCmdVelocity::onCmdVel, this);
        return true;
    }

private:
    Velocity<TYPE> currentCmdVelocity_;

    void onCmdVel(geometry_msgs::TwistConstPtr message)
    {
        set(TimeMath::time2number(message->header.stamp), message->linear.x, message->angular.z);
    }
};

} // namespace srs

#endif // ROSTAPCMDVELOCITY_HPP_
