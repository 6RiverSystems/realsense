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

#include <srslib_framework/ros/tap/RosTapPose.hpp>

namespace srs {

class RosTapInternal_InitialPose :
    public RosTapPose
{
public:
    RosTapInternal_InitialPose() :
        RosTapPose("/internal/command/initial_pose", "Internal initial pose")
    {}
};

} // namespace srs

#endif // ROSTAPINTERNAL_INITIALPOSE_HPP_
