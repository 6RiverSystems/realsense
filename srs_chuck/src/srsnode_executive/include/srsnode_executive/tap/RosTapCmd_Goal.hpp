/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef ROSTAPCMD_GOAL_HPP_
#define ROSTAPCMD_GOAL_HPP_

#include <string>
using namespace std;

#include <ros/ros.h>

#include <srslib_framework/ros/tap/RosTapPoseStamped.hpp>

namespace srs {

class RosTapCmd_Goal :
    public RosTapPoseStamped
{
public:
    RosTapCmd_Goal() :
        RosTapPoseStamped("/request/goal", "Command move")
    {}
};

} // namespace srs

#endif // ROSTAPCMD_GOAL_HPP_
