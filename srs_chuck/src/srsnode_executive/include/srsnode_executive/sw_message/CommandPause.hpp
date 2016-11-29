/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/ros/channel/ChannelBrainstemCmd_FreeSpin.hpp>

#include <srsnode_executive/sw_message/Command.hpp>

namespace srs {

class CommandPause : public Command
{
public:
    CommandPause() :
        Command(1)
    {}

    virtual ~CommandPause()
    {}

    bool execute(const vector<string>& params)
    {
        freeSpin_.publish(getSwitchParam(params, PARAM_1));

        return true;
    }

private:
    ChannelBrainstemCmd_FreeSpin freeSpin_;
};

} // namespace srs
