/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef ROSTAPCMD_INITIALPOSE_HPP_
#define ROSTAPCMD_INITIALPOSE_HPP_

#include <string>
using namespace std;

#include <ros/ros.h>

#include <srslib_framework/ros/tap/RosTapPoseWithCovariance.hpp>

namespace srs {

class RosTapCmd_InitialPose :
    public RosTapPoseWithCovariance
{
public:
    RosTapCmd_InitialPose() :
        RosTapPoseWithCovariance("/request/initial_pose", "Command 'Initial Pose' Tap")
    {}
};

} // namespace srs

#endif // ROSTAPCMD_INITIALPOSE_HPP_
