/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef VELOCITY_HPP_
#define VELOCITY_HPP_

#include <opencv2/opencv.hpp>

#include <platform/Object.hpp>

#include <framework/Math.hpp>

namespace srs {

template<typename TYPE = double>
struct Velocity : public Object
{
    Velocity(TYPE linear = TYPE(), TYPE angular = TYPE()) :
        linear(linear),
        angular(angular)
    {}

    ~Velocity()
    {}

    TYPE linear;
    TYPE angular;
};

} // namespace srs

#endif // VELOCITY_HPP_
