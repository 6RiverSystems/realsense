/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once
#include <string>
#include <srslib_framework/ros/channel/publisher/PublisherTimingData.hpp>
#include <srslib_framework/ros/topics/ChuckTopics.hpp>

namespace srs {

class ChannelTimingData :
    public PublisherTimingData
{
public:
    ChannelTimingData(std::string id = "none") :
        PublisherTimingData(ChuckTopics::monitoring::TIMING_DATA, id)
    {}
};

} // namespace srs
