/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef CHUCK_HPP_
#define CHUCK_HPP_

#include <srslib_framework/math/Math.hpp>
#include <srslib_framework/robotics/robot/RobotProfile.hpp>

namespace srs {

template<typename TYPE = double>
struct Chuck : RobotProfile
{
    constexpr static width() // [m]
    {
        return Math::inch2m<TYPE>(24.375);
    }

    constexpr static depth() // [m]
    {
        return Math::inch2m<TYPE>(38.474);
    }

//    constexpr static TYPE SIZE_WIDTH = Math::inch2m<TYPE>(24.375); // [m]
//    constexpr static TYPE SIZE_DEPTH = Math::inch2m<TYPE>(38.474); // [m]
//
//    constexpr static TYPE SIZE_WHEEL_DIAMETER = Math::inch2m<TYPE>(8); // [m]
//    constexpr static TYPE SIZE_WHEEL_DISTANCE = Math::inch2m<TYPE>(20.915); // [m]

//    MAX_VELOCITY = 3.0; % [m/s]
//    TRAVEL_VELOCITY = 2.0; % [m/s]
//
//    MAX_ACCELERATION = 2.0; % [m/s^2]
//    TRAVEL_ACCELERATION = 1.0; % [m/s^2]
//
//    MAX_OMEGA = deg2rad(120.0); % [rad/s]
//    TRAVEL_OMEGA = deg2rad(90.0); % [rad/s]
//
//    MAX_ALPHA = deg2rad(120.0); % [rad/s^2]
//    TRAVEL_ALPHA = deg2rad(90.0); % [rad/s^2]
//
//    ROBOT_NOISE = [0.0001, 0.0001, deg2rad(0.01), 0.0001, 0.0001]; % [m, m, rad]
};

} // namespace srs

#endif // CHUCK_HPP_
