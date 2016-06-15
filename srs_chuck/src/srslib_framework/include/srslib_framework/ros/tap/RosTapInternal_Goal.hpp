/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef ROSTAPINTERNALGOAL_HPP_
#define ROSTAPINTERNALGOAL_HPP_

#include <string>
using namespace std;

#include <ros/ros.h>

#include <srslib_framework/ros/tap/RosTapPose.hpp>

namespace srs {

class RosTapInternal_Goal :
    public RosTapPose
{
public:
    RosTapInternal_Goal() :
        RosTapPose("/internal/goal", "Internal goal")
    {}
};

} // namespace srs

#endif // ROSTAPINTERNALGOAL_HPP_
