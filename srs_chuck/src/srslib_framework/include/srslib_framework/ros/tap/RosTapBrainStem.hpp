/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef ROSTAPBRAINSTEM_HPP_
#define ROSTAPBRAINSTEM_HPP_

#include <string>
using namespace std;

#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <srslib_framework/math/TimeMath.hpp>
#include <srslib_framework/ros/RosTap.hpp>

namespace srs {

class RosTapBrainStem :
    public RosTap
{
public:
    RosTapBrainStem() :
        RosTap("/internal/drivers/brainstem/connected", "Brain Stem Status Tap"),
        connectionStateChanged_(false),
        isBrainStemConnected_(false),
        prevBrainStemConnected_(false)
    {}

    ~RosTapBrainStem()
    {
        disconnectTap();
    }

    bool isBrainStemConnected() const
    {
        return isBrainStemConnected_;
    }

    bool isConnectionStateChanged() const
    {
        return connectionStateChanged_;
    }

    void reset()
    {
        RosTap::reset();

        prevBrainStemConnected_ = false;
        set(TimeMath::time2number(ros::Time::now()), false);
    }

    void set(double arrivalTime, bool value)
    {
        isBrainStemConnected_ = value;

        connectionStateChanged_ = prevBrainStemConnected_ != isBrainStemConnected_;

        if (connectionStateChanged_)
        {
            ROS_INFO_STREAM("Brain stem changed the connection state from " <<
                prevBrainStemConnected_ << " to " << isBrainStemConnected_);
        }

        prevBrainStemConnected_ = isBrainStemConnected_;

        setNewData(true);
    }

protected:
    bool connect()
    {
        rosSubscriber_ = rosNodeHandle_.subscribe(getTopic(), 100,
            &RosTapBrainStem::onBrainStemConnected, this);

        return true;
    }

private:
    void onBrainStemConnected(std_msgs::BoolConstPtr message)
    {
        set(TimeMath::time2number(ros::Time::now()), message->data);
    }

    bool connectionStateChanged_;

    bool isBrainStemConnected_;

    bool prevBrainStemConnected_;
};

} // namespace srs

#endif // ROSTAPBRAINSTEM_HPP_
