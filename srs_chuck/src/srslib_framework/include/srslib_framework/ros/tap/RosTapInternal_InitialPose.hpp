/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef ROSTAPINTERNAL_INITIALPOSE_HPP_
#define ROSTAPINTERNAL_INITIALPOSE_HPP_

#include <string>
using namespace std;

#include <ros/ros.h>

#include <srslib_framework/ros/tap/RosTapMsgPose.hpp>

namespace srs {

class RosTapInternal_InitialPose :
    public RosTapMsgPose
{
public:
    RosTapInternal_InitialPose() :
        RosTapMsgPose("/internal/command/initial_pose", "Internal initial pose")
    {}
};

} // namespace srs

#endif // ROSTAPINTERNAL_INITIALPOSE_HPP_
