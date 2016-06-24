/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef ROSTAPCMD_MOVE_HPP_
#define ROSTAPCMD_MOVE_HPP_

#include <string>
using namespace std;

#include <ros/ros.h>

#include <srslib_framework/ros/tap/RosTapPoseStamped.hpp>

namespace srs {

class RosTapCmd_Move :
    public RosTapPoseStamped
{
public:
    RosTapCmd_Move() :
        RosTapPoseStamped("/request/move", "Command move")
    {}
};

} // namespace srs

#endif // ROSTAPCMD_MOVE_HPP_
