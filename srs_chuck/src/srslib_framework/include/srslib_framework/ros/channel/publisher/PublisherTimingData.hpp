/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
using namespace std;

#include <srslib_framework/TimingData.h>

#include <srslib_framework/platform/timing/TimingData.hpp>
#include <srslib_framework/ros/channel/publisher/Publisher.hpp>

namespace srs {

class PublisherTimingData :
    public Publisher<const TimingData&, srslib_framework::TimingData>
{
public:
    PublisherTimingData(string topic,
        unsigned int buffer = 100,
        bool latched = false,
        string nameSpace = "~") :
            Publisher(topic, buffer, latched, nameSpace)
    {}

    srslib_framework::TimingData convertData(const TimingData& data)
    {
        return TimingDataMessageFactory::data2Msg(data);
    }

private:
};

} // namespace srs
