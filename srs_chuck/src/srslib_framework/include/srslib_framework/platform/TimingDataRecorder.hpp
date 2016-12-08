/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
#include <utility>
#include <srslib_framework/platform/StopWatch.hpp>
#include <srslib_framework/ros/message/TimingDataMessageFactory.hpp>
#include <srslib_framework/ros/channel/ChannelTimingData.hpp>
#include <ros/ros.h>

namespace srs {


class TimingDataRecorder
{

public:
    TimingDataRecorder(std::string id, double sampleSetLength = 1.0) :
        id_(id),
        channel_(id),
        sampleSetLength_(sampleSetLength)
    {};

    ~TimingDataRecorder()
    {};

    void startSample()
    {
        if (sampleSetStartTime_ < 0)
        {
            // Starting a new sample set, so record the start time
            sampleSetStartTime_ = ros::Time::now().toSec();
        }
        stopWatch_.reset();
    };

    void stopSample()
    {
        if (sampleSetStartTime_ < 0)
        {
            return;
        }

        double currentTime = ros::Time::now().toSec();
        samples_.push_back(std::make_pair(stopWatch_.elapsed(), currentTime));
        if (currentTime - sampleSetStartTime_ >= sampleSetLength_)
        {
            // Publish.
            channel_.publish(TimingDataMessageFactory::buildMessage(sampleSetStartTime_, currentTime, samples_));

            logData();
            reset();
        }
    }
private:
    void logData()
    {

        double meanVal = 0;
        double maxVal = 0;
        if (samples_.size() > 0)
        {
            for (auto s : samples_)
            {
                meanVal += s.first;
                if (s.first > maxVal)
                {
                    maxVal = s.first;
                }
            }
            meanVal /= samples_.size();
        }
        ROS_DEBUG_NAMED("timing-data", "Timing for: %s.  Mean: %f, max: %f", id_.c_str(), meanVal, maxVal);
    }

    void reset()
    {
        sampleSetStartTime_ = -1.0;
        samples_.clear();
    }

    StopWatch stopWatch_;

    ChannelTimingData channel_;
    std::string id_;
    double sampleSetLength_ = 1.0;
    double sampleSetStartTime_ = -1.0;
    std::vector<std::pair<double, double>> samples_;
};


} // namespace srs
