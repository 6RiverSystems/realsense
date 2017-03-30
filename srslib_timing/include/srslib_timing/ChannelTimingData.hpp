/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once
#include <string>
#include <srslib_timing/PublisherTimingData.hpp>

namespace srs {

class ChannelTimingData :
    public PublisherTimingData
{
public:
    ChannelTimingData() :
        PublisherTimingData("/monitoring/timing_data")
    {}
};

} // namespace srs
