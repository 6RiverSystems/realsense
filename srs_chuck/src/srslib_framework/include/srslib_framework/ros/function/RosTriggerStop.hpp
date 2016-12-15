/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef ROSTRIGGERSTOP_HPP_
#define ROSTRIGGERSTOP_HPP_

#include <string>
using namespace std;

#include <srslib_framework/ros/service/RosServiceTrigger.hpp>

namespace srs {

class RosTriggerStop :
    public RosServiceTrigger
{
public:
    RosTriggerStop() :
        RosServiceTrigger("stop", "Trigger: Stop")
    {}
};

} // namespace srs

#endif // ROSTRIGGERSHUTDOWN_HPP_
