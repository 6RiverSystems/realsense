/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef FILTERSTATE_HPP_
#define FILTERSTATE_HPP_

#include <opencv2/opencv.hpp>

#include <platform/Object.hpp>
#include <platform/Ocv2Base.hpp>

namespace srs {

template<unsigned int STATE_SIZE = 5, int TYPE = CV_64F>
struct FilterState : public Object
{
    typedef typename Ocv2Base<TYPE>::BaseType BaseType;

    FilterState()
    {}

    ~FilterState()
    {}

    virtual cv::Mat getStateVector() = 0;
};

} // namespace srs

#endif // FILTERSTATE_HPP_
