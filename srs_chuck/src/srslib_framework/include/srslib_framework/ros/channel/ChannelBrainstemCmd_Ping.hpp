/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/ros/channel/publisher/PublisherBoolean.hpp>
#include <srslib_framework/chuck/ChuckTopics.hpp>

namespace srs {

class ChannelBrainstemCmd_Ping :
    public PublisherBoolean
{
public:
    ChannelBrainstemCmd_Ping() :
    	PublisherBoolean(ChuckTopics::driver::BRAINSTEM_CMD_PING)
    {}
};

} // namespace srs