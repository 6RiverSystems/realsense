/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <std_msgs/String.h>

#include <srslib_framework/platform/SoftwareMessageHandler.hpp>
#include <srslib_framework/ros/tap/subscriber/Subscriber.hpp>
#include <srslib_framework/ros/tap/subscriber/Observer.hpp>
#include <srslib_framework/ros/tap/TapExecutiveCmd_Cl.hpp>

#include <srsnode_executive/sw_message/Command.hpp>
#include <srsnode_executive/sw_message/CommandPause.hpp>

namespace srs {

class Executive;

class CommandLineHandler :
    public SoftwareMessageHandler<Executive>,
    public Observer<Subscriber<std_msgs::String>>
{
public:
    CommandLineHandler(Executive* owner);

    virtual ~CommandLineHandler()
    {}

    void notified(Subscriber<std_msgs::String>* subject);

private:
    void processString(const string& message);

    std::map<std::string, Command*> commandMap_;

    CommandPause commandPause_;

    TapExecutiveCmd_Cl tapCl_;
};

} // namespace srs
