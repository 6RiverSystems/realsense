/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/ros/channel/ChannelBrainstemCmd_SetMotionState.hpp>

#include <srsnode_executive/sw_message/Command.hpp>

namespace srs {

class Executive;

class CommandPause : public Command
{
public:
    CommandPause(Executive* owner);

    virtual ~CommandPause()
    {}

    bool execute(const vector<string>& params);

private:
    ChannelBrainstemCmd_SetMotionState setMotionState_;
};

} // namespace srs
