/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef ROSTAPJOYADAPTER_HPP_
#define ROSTAPJOYADAPTER_HPP_

#include <algorithm>
using namespace std;

#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>

#include <srslib_framework/ros/RosTap.hpp>
#include <srslib_framework/robotics/Velocity.hpp>

namespace srs {

class RosTapJoyAdapter :
    public RosTap
{
public:
    RosTapJoyAdapter() :
        RosTap("", "Joy Adapter Tap"),
        currentCustomActionState_(false),
        currentEmergencyState_(false),
        currentLatchState_(false)
    {}

    ~RosTapJoyAdapter()
    {
        disconnectTap();
    }

    bool getCustomActionState()
    {
        setNewData(false);
        return currentCustomActionState_;
    }

    bool getEmergencyState()
    {
        setNewData(false);
        return currentEmergencyState_;
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

    void reset()
    {
        RosTap::reset();

        currentLatchState_ = false;
        currentEmergencyState_ = false;
        currentCustomActionState_ = false;
        currentVelocity_ = Velocity<>();
    }

protected:
    bool connect()
    {
        rosSubscriber_ = rosNodeHandle_.subscribe("/internal/sensors/joystick/latched", 1,
            &RosTapJoyAdapter::onJoyLatched, this);
        subJoyVelocity_ = rosNodeHandle_.subscribe("/internal/sensors/joystick/velocity", 1,
            &RosTapJoyAdapter::onJoyVelocity, this);
        subJoyEmergency_ = rosNodeHandle_.subscribe("/internal/sensors/joystick/emergency", 1,
            &RosTapJoyAdapter::onJoyEmergency, this);
        subJoyCustomAction_ = rosNodeHandle_.subscribe("/internal/sensors/joystick/custom_action", 1,
            &RosTapJoyAdapter::onJoyCustomAction, this);

        return true;
    }

private:
    void onJoyCustomAction(const std_msgs::BoolConstPtr& message)
    {
        currentCustomActionState_ = message->data;
        setNewData(true);
    }

    void onJoyEmergency(const std_msgs::BoolConstPtr& message)
    {
        currentEmergencyState_ = message->data;
        setNewData(true);
    }

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

    bool currentCustomActionState_;
    bool currentEmergencyState_;
    bool currentLatchState_;
    Velocity<> currentVelocity_;

    ros::Subscriber subJoyCustomAction_;
    ros::Subscriber subJoyEmergency_;
    ros::Subscriber subJoyVelocity_;
};

} // namespace srs

#endif // ROSTAPJOYADAPTER_HPP_
