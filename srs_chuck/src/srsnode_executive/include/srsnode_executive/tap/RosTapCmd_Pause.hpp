/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef ROSTAPCMD_PAUSE_HPP_
#define ROSTAPCMD_PAUSE_HPP_

#include <string>
using namespace std;

#include <srslib_framework/ros/tap/RosTapBool.hpp>

namespace srs {

class RosTapCmd_Pause :
    public RosTapBool
{
public:
    RosTapCmd_Pause() :
        RosTapBool("/cmd/pause", "Command 'Pause' Tap")
    {}
};

} // namespace srs

#endif // ROSTAPCMD_PAUSE_HPP_
