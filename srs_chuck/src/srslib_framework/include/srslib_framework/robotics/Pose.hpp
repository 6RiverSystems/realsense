/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef POSE_HPP_
#define POSE_HPP_

#include <opencv2/opencv.hpp>

#include <srslib_framework/platform/Object.hpp>
#include <srslib_framework/math/Math.hpp>

namespace srs {

template<typename TYPE = double>
struct Pose : public Object
{
    Pose(TYPE x = TYPE(), TYPE y = TYPE(), TYPE theta = TYPE()) :
        x(x),
        y(y),
        theta(theta)
    {}

    ~Pose()
    {}

    void setThetaDegrees(TYPE deg)
    {
        theta = Math::deg2rad<TYPE>(deg);
    }

    TYPE x;
    TYPE y;
    TYPE theta;
};

} // namespace srs

#endif // POSE_HPP_
