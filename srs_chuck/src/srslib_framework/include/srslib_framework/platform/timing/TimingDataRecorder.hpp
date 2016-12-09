/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
#include <utility>
#include <srslib_framework/platform/timing/StopWatch.hpp>
#include <srslib_framework/platform/timing/TimingData.hpp>

#include <srslib_framework/ros/message/TimingDataMessageFactory.hpp>
#include <srslib_framework/ros/channel/ChannelTimingData.hpp>
#include <ros/ros.h>

namespace srs {


class TimingDataRecorder
{
public:
    TimingDataRecorder(std::string id, double sampleSetLength = 1.0) :
        timingData_(id),
        sampleSetLength_(sampleSetLength)
    {};

    ~TimingDataRecorder()
    {};

    void startSample()
    {
        if (!timingData_.hasStarted())
        {
            // Starting a new sample set, so record the start time
            timingData_.setStartTime(ros::Time::now().toSec());
        }
        stopWatch_.reset();
        activeSample_ = true;
    };

    void stopSample()
    {
        if (!timingData_.hasStarted() || !activeSample_)
        {
            return;
        }

        double currentTime = ros::Time::now().toSec();
        timingData_.addSample(currentTime, stopWatch_.elapsed());
        activeSample_ = false;

        if (currentTime - timingData_.getStartTime() >= sampleSetLength_)
        {
            timingData_.setEndTime(currentTime);
            // Publish.
            channel_.publish(timingData_);

            // Log data.
            ROS_DEBUG_NAMED("timing-data", "%s", timingData_.getDataForLog().c_str());

            // Reset
            timingData_.reset();
        }
    }

    std::string getId()
    {
        return timingData_.getId();
    }

private:
    StopWatch stopWatch_;
    TimingData timingData_;
    ChannelTimingData channel_;

    double sampleSetLength_ = 1.0;
    bool activeSample_ = false;
};


} // namespace srs
