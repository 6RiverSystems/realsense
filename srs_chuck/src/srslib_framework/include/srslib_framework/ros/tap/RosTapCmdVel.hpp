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
        setNewData(false, 0);
        return currentCmdVel_;
    }

//    Velocity<TYPE> getPreviousData()
//    {
//        return currentCmdVel_;
//    }
//
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
        ros::Time timestamp = ros::Time::now();

        currentCmdVel_.linear = message->linear.x;
        currentCmdVel_.angular = message->angular.z;

        ROS_DEBUG_THROTTLE(0.5f, "RosTapCmdVel | linear = %f, angular = %f",
            currentCmdVel_.linear,
            currentCmdVel_.angular);

        setNewData(true, timestamp.nsec);
    }
};

} // namespace srs

#endif // ROSTAPCMDVEL_HPP_
