/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <opencv2/opencv.hpp>

#include <srslib_framework/math/Ocv2Base.hpp>

namespace srs {

template<unsigned int STATE_SIZE = 5, int TYPE = CV_64F>
struct FilterState
{
    typedef typename Ocv2Base<TYPE>::BaseType BaseType;

    FilterState()
    {}

    virtual ~FilterState()
    {}

    virtual cv::Mat getVectorForm() = 0;
};

} // namespace srs