/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef ROSTRIGGERPAUSE_HPP_
#define ROSTRIGGERPAUSE_HPP_

#include <string>
using namespace std;

#include <srslib_framework/ros/service/RosServiceBoolTrigger.hpp>

namespace srs {

class RosTriggerPause :
    public RosServiceBoolTrigger
{
public:
    RosTriggerPause() :
        RosServiceBoolTrigger("pause", "Trigger: Pause")
    {}
};

} // namespace srs

#endif // ROSTRIGGERPAUSE_HPP_
