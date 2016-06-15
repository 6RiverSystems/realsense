/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef ROSTAPINTERNAL_GOALARRIVED_HPP_
#define ROSTAPINTERNAL_GOALARRIVED_HPP_

#include <string>
using namespace std;

#include <srslib_framework/ros/tap/RosTapBool.hpp>

namespace srs {

class RosTapInternal_GoalArrived :
    public RosTapBool
{
public:
    RosTapInternal_GoalArrived() :
        RosTapBool("/internal/state/current_goal/arrived", "ARRIVED to goal")
    {}
};

} // namespace srs

#endif // ROSTAPINTERNAL_GOALARRIVED_HPP_
