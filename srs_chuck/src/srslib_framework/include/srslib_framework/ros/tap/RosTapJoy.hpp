/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef ROSTAPJOY_HPP_
#define ROSTAPJOY_HPP_

#include <string>
using namespace std;

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

#include <srslib_framework/ros/RosTap.hpp>
#include <srslib_framework/robotics/Velocity.hpp>

namespace srs {

template<typename TYPE = double>
class RosTapJoy :
    public RosTap
{
public:
    RosTapJoy() :
        RosTap("Joy Tap"),
        currentVelocity_()
    {}

    ~RosTapJoy()
    {
        disconnectTap();
    }

    Velocity<TYPE> getCurrentVelocity()
    {
        setNewData(false, 0);
        return currentVelocity_;
    }

protected:
    bool connect()
    {
        rosSubscriber_ = rosNodeHandle_.subscribe("/joy", 10, &RosTapJoy::onJoy, this);
        return true;
    }

private:
    Velocity<TYPE> currentVelocity_;

    void onJoy(const sensor_msgs::Joy::ConstPtr& message)
    {
        currentVelocity_.linear = message->axes[1];
        currentVelocity_.angular = message->axes[0];

        setNewData(true, ros::Time::now().nsec);
    }
};

} // namespace srs

#endif // ROSTAPJOY_HPP_