/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/math/MeasurementMath.hpp>
#include <srslib_framework/math/AngleMath.hpp>

namespace srs {

namespace ChuckLimits
{
    static constexpr float PHYSICAL_MAX_ANGULAR = 2.000; // [rad/s]
    static constexpr float PHYSICAL_MAX_LINEAR = 1.500; // [rad/s]
};

} // namespace srs
