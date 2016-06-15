/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef ROSTAPCMD_SHUTDOWN_HPP_
#define ROSTAPCMD_SHUTDOWN_HPP_

#include <string>
using namespace std;

#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <srslib_framework/math/TimeMath.hpp>
#include <srslib_framework/ros/RosTap.hpp>

namespace srs {

class RosTapCmd_Shutdown :
    public RosTap
{
public:
    RosTapCmd_Shutdown() :
        RosTap("Command 'Shutdown' Tap"),
        currentShutdown_(false)
    {}

    ~RosTapCmd_Shutdown()
    {
        disconnectTap();
    }

    bool getShutdown()
    {
        setNewData(false);
        return currentShutdown_;
    }

    bool isShutdownRequested()
    {
        return newDataAvailable() && getShutdown();
    }

    void reset()
    {
        set(TimeMath::time2number(ros::Time::now()), false);
    }

    void set(double arrivalTime, bool shutdown)
    {
        currentShutdown_ = pause;
        setNewData(true);
    }

protected:
    bool connect()
    {
        rosSubscriber_ = rosNodeHandle_.subscribe("/cmd/shutdown", 10, &RosTapCmd_Shutdown::onShutdown, this);
        return true;
    }

private:
    void onShutdown(const std_msgs::BoolConstPtr message)
    {
        set(TimeMath::time2number(ros::Time::now()), message->data);
    }

    bool currentShutdown_;
};

} // namespace srs

#endif // ROSTAPCMD_SHUTDOWN_HPP_
