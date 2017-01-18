/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
using namespace std;

#include <ros/ros.h>
#include <srslib_timing/TimingData.h>

#include <srslib_timing/TimingData.hpp>

namespace srs {

class PublisherTimingData :
    public Publisher<const TimingData&, srslib_framework::TimingData>
{
public:
    PublisherTimingData(string topic,
        unsigned int buffer = 100,
        bool latched = false,
        string nameSpace = "~") : topic_(topic)
    {
        rosNodeHandle_ = ros::NodeHandle(nameSpace);
        dataPublisher_ = rosNodeHandle_.advertise<srslib_timing::TimingData>(topic_, queueLength, latched);
    }

    std::string getTopic() const
    {
        return topic_;
    }

    virtual void publish(TimingData data)
    {
        publishMessage(convertData(data));
    }

    srslib_timing::TimingData convertData(const TimingData& data)
    {
        return TimingDataMessageFactory::data2Msg(data);
    }

private:
    void publishMessage(const srslib_timing::TimingData& message)
    {
        dataPublisher_.publish(message);
    }

    ros::Publisher dataPublisher_;
    ros::NodeHandle rosNodeHandle_;
    std::string topic_
};

} // namespace srs
