/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef ROSTAPINTERNAL_ROBOTPOSE_HPP_
#define ROSTAPINTERNAL_ROBOTPOSE_HPP_

#include <srslib_framework/ros/tap/RosTapMsgPose.hpp>

namespace srs {

class RosTapInternal_RobotPose :
    public RosTapMsgPose
{
public:
    RosTapInternal_RobotPose() :
        RosTapMsgPose("/internal/state/robot/pose", "Internal robot pose")
    {}
};

} // namespace srs

#endif // ROSTAPINTERNAL_ROBOTPOSE_HPP_
