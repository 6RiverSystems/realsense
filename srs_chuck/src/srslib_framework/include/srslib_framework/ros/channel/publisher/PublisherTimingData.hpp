/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
using namespace std;

#include <srslib_framework/TimingData.h>

#include <srslib_framework/ros/channel/publisher/Publisher.hpp>

namespace srs {

class PublisherTimingData :
    public Publisher<const srslib_framework::TimingData&, srslib_framework::TimingData>
{
public:
    PublisherTimingData(string topic,
        string id,
        unsigned int buffer = 100,
        bool latched = false,
        string nameSpace = "~") :
            Publisher(topic, buffer, latched, nameSpace),
            id_(id)
    {}

    srslib_framework::TimingData convertData(const srslib_framework::TimingData& data)
    {
        srslib_framework::TimingData out = data;
        out.id = id_;
        return out;
    }

private:
    string id_;
};

} // namespace srs
