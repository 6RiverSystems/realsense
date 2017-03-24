/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <opencv2/opencv.hpp>

#include <srslib_framework/math/Ocv2Base.hpp>

namespace srs {

class Measurement
{
public:
    Measurement()
    {}

    virtual ~Measurement()
    {}
};

} // namespace srs
