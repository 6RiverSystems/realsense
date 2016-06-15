/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef ROSTAPCMD_PAUSE_HPP_
#define ROSTAPCMD_PAUSE_HPP_

#include <string>
using namespace std;

#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <srslib_framework/math/TimeMath.hpp>
#include <srslib_framework/ros/RosTap.hpp>

namespace srs {

class RosTapCmd_Pause :
    public RosTap
{
public:
    RosTapCmd_Pause() :
        RosTap("Command 'Pause' Tap"),
        currentPause_(false)
    {}

    ~RosTapCmd_Pause()
    {
        disconnectTap();
    }

    bool getPause()
    {
        setNewData(false);
        return currentPause_;
    }

    bool isPauseRequested()
    {
        return newDataAvailable() && getPause();
    }

    void reset()
    {
        set(TimeMath::time2number(ros::Time::now()), false);
    }

    void set(double arrivalTime, bool pause)
    {
        currentPause_ = pause;
        setNewData(true);
    }

protected:
    bool connect()
    {
        rosSubscriber_ = rosNodeHandle_.subscribe("/cmd/pause", 10, &RosTapCmd_Pause::onPause, this);
        return true;
    }

private:
    void onPause(const std_msgs::BoolConstPtr message)
    {
        set(TimeMath::time2number(ros::Time::now()), message->data);
    }

    bool currentPause_;
};

} // namespace srs

#endif // ROSTAPCMD_PAUSE_HPP_
