/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef BASE2OCV_HPP_
#define BASE2OCV_HPP_

#include <limits>
using namespace std;

#include <opencv2/opencv.hpp>

namespace srs {

template<typename TYPE>
class Base2Ocv;

template<>
class Base2Ocv<double>
{
public:
    constexpr static int OCV_TYPE = CV_64F;
};

template<>
class Base2Ocv<float>
{
public:
    constexpr static int OCV_TYPE = CV_32F;
};

} // namespace srs

#endif // BASE2OCV_HPP_
