/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/ros/channel/publisher/PublisherString.hpp>
#include <srslib_framework/chuck/ChuckTopics.hpp>

namespace srs {

class ChannelExecutiveCmd_Cl :
    public PublisherString
{
public:
    ChannelExecutiveCmd_Cl() :
        PublisherString(ChuckTopics::node::EXECUTIVE_CMD_CL)
    {}
};

} // namespace srs
