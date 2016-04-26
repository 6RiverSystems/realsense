/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef ROSVELCMD_HPP_
#define ROSVELCMD_HPP_

#include <string>
using namespace std;

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <srslib_framework/ros/RosTap.hpp>

#include "VelCmd.hpp"

namespace srs {

class RosVelCmd :
    public RosTap
{
public:
    RosVelCmd() :
        RosTap("CmdVel Tap"),
        currentCommand_()
    {}

    ~RosVelCmd()
    {
        disconnectTap();
    }

    VelCmd<> getCurrentData()
    {
        setNewData(false, 0);
        return currentCommand_;
    }

    VelCmd<> getPreviousData()
    {
        return currentCommand_;
    }

protected:
    bool connect();

private:
    VelCmd<> currentCommand_;

    void onCmdVel(geometry_msgs::TwistConstPtr message);
};

} // namespace srs

#endif // ROSVELCMD_HPP_
