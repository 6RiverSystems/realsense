/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef ROSTAPCMD_SHUTDOWN_HPP_
#define ROSTAPCMD_SHUTDOWN_HPP_

#include <string>
using namespace std;

#include <srslib_framework/ros/tap/RosTapBool.hpp>

namespace srs {

class RosTapCmd_Shutdown :
    public RosTapBool
{
public:
    RosTapCmd_Shutdown() :
        RosTapBool("/cmd/shutdown", "Command 'Shutdown' Tap")
    {}
};

} // namespace srs

#endif // ROSTAPCMD_SHUTDOWN_HPP_
