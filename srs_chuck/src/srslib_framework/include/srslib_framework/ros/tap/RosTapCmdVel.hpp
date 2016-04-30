/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef ROSTAPCMDVEL_HPP_
#define ROSTAPCMDVEL_HPP_

#include <string>
using namespace std;

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <srslib_framework/ros/RosTap.hpp>
#include <srslib_framework/robotics/Velocity.hpp>

namespace srs {

template<typename TYPE = double>
class RosTapCmdVel :
    public RosTap
{
public:
    RosTapCmdVel() :
        RosTap("CmdVel Tap"),
        currentCmdVel_()
    {}

    ~RosTapCmdVel()
    {
        disconnectTap();
    }

    Velocity<TYPE> getCurrentData()
    {
        setNewData(false);
        return currentCmdVel_;
    }

protected:
    bool connect()
    {
        rosSubscriber_ = rosNodeHandle_.subscribe("/cmd_vel", 1000, &RosTapCmdVel::onCmdVel, this);
        return true;
    }

private:
    Velocity<TYPE> currentCmdVel_;

    void onCmdVel(geometry_msgs::TwistConstPtr message)
    {
        currentCmdVel_ = Velocity<TYPE>(message->linear.x, message->angular.z);
        setNewData(true);
    }
};

} // namespace srs

#endif // ROSTAPCMDVEL_HPP_
