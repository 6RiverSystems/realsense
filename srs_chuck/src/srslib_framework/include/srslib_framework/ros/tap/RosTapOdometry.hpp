/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef ROSTAPODOMETRY_HPP_
#define ROSTAPODOMETRY_HPP_

#include <string>
using namespace std;

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>

#include <srslib_framework/math/TimeMath.hpp>
#include <srslib_framework/ros/RosTap.hpp>
#include <srslib_framework/robotics/Odometry.hpp>

namespace srs {

class RosTapOdometry :
    public RosTap
{
public:
    typedef typename Odometry<>::BaseType BaseType;

    RosTapOdometry() :
        RosTap("Odometry Tap"),
        currentOdometry_()
    {}

    ~RosTapOdometry()
    {
        disconnectTap();
    }

    Odometry<> getOdometry()
    {
        setNewData(false);
        return currentOdometry_;
    }

    void reset()
    {
        RosTap::reset();

        set(TimeMath::time2number(ros::Time::now()), 0.0, 0.0);
    }

    void set(double arrivalTime, BaseType linear, BaseType angular)
    {
        currentOdometry_ = Odometry<>(arrivalTime, linear, angular);
        setNewData(true);
    }

protected:
    bool connect()
    {
        rosSubscriber_ = rosNodeHandle_.subscribe("/internal/sensors/odometry/raw", 100,
            &RosTapOdometry::onOdometry, this);

        return true;
    }

private:
    void onOdometry(geometry_msgs::TwistStampedConstPtr message)
    {
        set(TimeMath::time2number(message->header.stamp),
            static_cast<double>(message->twist.linear.x),
            static_cast<double>(message->twist.angular.z));
    }

    Odometry<> currentOdometry_;
};

} // namespace srs

#endif // ROSTAPODOMETRY_HPP_
