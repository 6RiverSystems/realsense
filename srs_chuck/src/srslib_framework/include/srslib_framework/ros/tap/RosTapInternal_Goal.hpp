/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef ROSTAPINTERNALGOAL_HPP_
#define ROSTAPINTERNALGOAL_HPP_

#include <srslib_framework/ros/tap/RosTapMsgPose.hpp>

namespace srs {

class RosTapInternal_Goal :
    public RosTapMsgPose
{
public:
    RosTapInternal_Goal() :
        RosTapMsgPose("/internal/command/goal", "Internal goal command")
    {}
};

} // namespace srs

#endif // ROSTAPINTERNALGOAL_HPP_
