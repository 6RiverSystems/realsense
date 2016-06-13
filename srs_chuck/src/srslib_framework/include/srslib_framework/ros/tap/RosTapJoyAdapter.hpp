/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef ROSTAPJOYADAPTER_HPP_
#define ROSTAPJOYADAPTER_HPP_

#include <string>
#include <algorithm>
using namespace std;

#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <srslib_framework/ros/RosTap.hpp>
#include <srslib_framework/robotics/Velocity.hpp>

namespace srs {

class RosTapJoyAdapter :
    public RosTap
{
public:
    RosTapJoyAdapter(ros::NodeHandle rosHandle) :
        RosTap(rosHandle, "Joy Adapter Tap"),
        currentLatchState_(false)
    {}

    ~RosTapJoyAdapter()
    {
        disconnectTap();
    }

    bool getLatchState()
    {
        setNewData(false);
        return currentLatchState_;
    }

    Velocity<> getVelocity()
    {
        setNewData(false);
        return currentVelocity_;
    }

protected:
    bool connect()
    {
        rosSubscriber_ = rosNodeHandle_.subscribe("/srsnode_joystick_adapter/latched", 1,
            &RosTapJoyAdapter::onJoyLatched, this);
        subJoyVelocity_ = rosNodeHandle_.subscribe("/srsnode_joystick_adapter/velocity", 1,
            &RosTapJoyAdapter::onJoyVelocity, this);
        return true;
    }

private:
    bool currentLatchState_;
    Velocity<> currentVelocity_;

    ros::Subscriber subJoyVelocity_;

    void onJoyLatched(const std_msgs::BoolConstPtr& message)
    {
        currentLatchState_ = message->data;
        setNewData(true);
    }

    void onJoyVelocity(const geometry_msgs::TwistConstPtr& message)
    {
        currentVelocity_ = Velocity<>(message->linear.x, message->angular.z);
        setNewData(true);
    }
};

} // namespace srs

#endif // ROSTAPJOYADAPTER_HPP_
