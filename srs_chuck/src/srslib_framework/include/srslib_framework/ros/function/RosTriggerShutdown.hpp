/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef ROSTRIGGERSHUTDOWN_HPP_
#define ROSTRIGGERSHUTDOWN_HPP_

#include <string>
using namespace std;

#include <srslib_framework/ros/function/RosServiceTrigger.hpp>

namespace srs {

class RosTriggerShutdown :
    public RosServiceTrigger
{
public:
    RosTriggerShutdown() :
        RosServiceTrigger("shutdown", "Trigger: Shutdown")
    {}
};

} // namespace srs

#endif // ROSTRIGGERSHUTDOWN_HPP_
