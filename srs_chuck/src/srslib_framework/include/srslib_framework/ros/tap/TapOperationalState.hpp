/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <algorithm>
using namespace std;

#include <srslib_framework/ros/tap/subscriber/SubscriberRobotState.hpp>
#include <srslib_framework/chuck/ChuckTopics.hpp>

namespace srs {

class TapOperationalState :
    public SubscriberRobotState
{
public:
    TapOperationalState() :
        SubscriberRobotState(ChuckTopics::driver::BRAINSTEM_OPERATIONAL_STATE, 1)
    {}

    ~TapOperationalState()
    {}

    bool getFreeSpin()
    {
        return pop().freeSpin;
    }
};

} // namespace srs
