/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef OCV2BASE_HPP_
#define OCV2BASE_HPP_

#include <opencv2/opencv.hpp>

namespace srs {

template<int OPENCV_TYPE>
class Ocv2Base;

template<>
class Ocv2Base<CV_64F>
{
public:
    typedef double BaseType;
};

template<>
class Ocv2Base<CV_32F>
{
public:
    typedef float BaseType;
};

} // namespace srs

#endif // OCV2BASE_HPP_
