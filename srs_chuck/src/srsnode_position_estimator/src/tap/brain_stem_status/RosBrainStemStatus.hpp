/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef ROSBRAINSTEMSTATUS_HPP_
#define ROSBRAINSTEMSTATUS_HPP_

#include <string>
using namespace std;

#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <srslib_framework/ros/RosTap.hpp>

namespace srs {

class RosBrainStemStatus :
    public RosTap
{
public:
    RosBrainStemStatus() :
        RosTap("Brain Stem Status Tap"),
        connectionStateChanged_(false),
        isBrainStemConnected_(false),
        prevBrainStemConnected_(false)
    {}

    ~RosBrainStemStatus()
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
    bool connect();

private:
    bool connectionStateChanged_;
    bool isBrainStemConnected_;
    bool prevBrainStemConnected_;

    void onBrainStemConnected(std_msgs::BoolConstPtr message);
};

} // namespace srs

#endif // ROSBRAINSTEMSTATUS_HPP_
