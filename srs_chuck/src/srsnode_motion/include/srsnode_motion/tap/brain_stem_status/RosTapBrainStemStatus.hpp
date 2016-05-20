/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef ROSTAPBRAINSTEMSTATUS_HPP_
#define ROSTAPBRAINSTEMSTATUS_HPP_

#include <string>
using namespace std;

#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <srslib_framework/ros/RosTap.hpp>

namespace srs {

class RosTapBrainStemStatus :
    public RosTap
{
public:
    RosTapBrainStemStatus(ros::NodeHandle rosHandle) :
        RosTap(rosHandle, "Brain Stem Status Tap"),
        connectionStateChanged_(false),
        isBrainStemConnected_(false),
        prevBrainStemConnected_(false)
    {}

    ~RosTapBrainStemStatus()
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

protected:
    bool connect()
    {
        rosSubscriber_ = rosNodeHandle_.subscribe("/brain_stem/connected", 100,
            &RosTapBrainStemStatus::onBrainStemConnected, this);

        return true;
    }

private:
    bool connectionStateChanged_;
    bool isBrainStemConnected_;
    bool prevBrainStemConnected_;

    void onBrainStemConnected(std_msgs::BoolConstPtr message)
    {
        isBrainStemConnected_ = message->data;
        connectionStateChanged_ = prevBrainStemConnected_ != isBrainStemConnected_;

        if (connectionStateChanged_)
        {
            ROS_INFO_STREAM("Brain stem changed the connection state from " <<
                prevBrainStemConnected_ << " to " << isBrainStemConnected_);
        }

        prevBrainStemConnected_ = isBrainStemConnected_;
        setNewData(true);
    }
};

} // namespace srs

#endif // ROSTAPBRAINSTEMSTATUS_HPP_
