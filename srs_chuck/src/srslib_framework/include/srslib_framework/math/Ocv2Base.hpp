/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef OCV2BASE_HPP_
#define OCV2BASE_HPP_

#include <limits>
using namespace std;

namespace srs {

template<int OPENCV_TYPE>
class Ocv2Base;

template<>
class Ocv2Base<CV_64F>
{
public:
    typedef double BaseType;
    constexpr static double MIN_VALUE = numeric_limits<double>::min();
    constexpr static double MAX_VALUE = numeric_limits<double>::max();
};

template<>
class Ocv2Base<CV_32F>
{
public:
    typedef float BaseType;
    constexpr static float MIN_VALUE = numeric_limits<float>::min();
    constexpr static float MAX_VALUE = numeric_limits<float>::max();
};

} // namespace srs

#endif // OCV2BASE_HPP_
