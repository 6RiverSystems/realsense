/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef POSE_HPP_
#define POSE_HPP_

#include <opencv2/opencv.hpp>

#include <platform/Object.hpp>
#include <framework/Math.hpp>

namespace srs {

template<typename TYPE>
struct Pose : public Object
{
    Pose(TYPE x = 0.0f, TYPE y = 0.0f, TYPE theta = 0.0f) :
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
