/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef ROBOTPROFILE_HPP_
#define ROBOTPROFILE_HPP_

#include <srslib_framework/math/Math.hpp>

namespace srs {

template<typename TYPE = double>
struct RobotProfile
{
    constexpr static width() = 0; // [m]
    constexpr static depth() = 0; // [m]
};

} // namespace srs

#endif // ROBOTPROFILE_HPP_
