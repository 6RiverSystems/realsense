/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef ROSTAPBOOL_HPP_
#define ROSTAPBOOL_HPP_

#include <string>
using namespace std;

#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <srslib_framework/math/TimeMath.hpp>
#include <srslib_framework/ros/RosTap.hpp>

namespace srs {

class RosTapBool :
    public RosTap
{
public:
    RosTapBool(string topic, string description = "Bool Tap") :
        RosTap(topic, description),
        currentBool_(false)
    {}

    ~RosTapBool()
    {
        disconnectTap();
    }

    bool getBool()
    {
        setNewData(false);
        return currentBool_;
    }

    bool isNewValueTrue()
    {
        return newDataAvailable() && getBool();
    }

    void reset()
    {
        set(TimeMath::time2number(ros::Time::now()), false);
    }

    void set(double arrivalTime, bool value)
    {
        currentBool_ = value;
        setNewData(true);
    }

protected:
    bool connect()
    {
        rosSubscriber_ = rosNodeHandle_.subscribe(getTopic(), 10, &RosTapBool::onBool, this);
        return true;
    }

private:
    void onBool(const std_msgs::BoolConstPtr message)
    {
        set(TimeMath::time2number(ros::Time::now()), message->data);
    }

    bool currentBool_;
};

} // namespace srs

#endif // ROSTAPBOOL_HPP_
