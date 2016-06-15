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

#include <srslib_framework/ros/tap/RosTapPose.hpp>

namespace srs {

class RosTapCmd_Goal :
    public RosTapPose
{
public:
    RosTapCmd_Goal() :
        RosTapPose("/request/goal", "Command goal")
    {}
};

} // namespace srs

#endif // ROSTAPCMD_GOAL_HPP_
