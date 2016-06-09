/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef MEASUREMENT_HPP_
#define MEASUREMENT_HPP_

#include <opencv2/opencv.hpp>

#include <srslib_framework/math/Ocv2Base.hpp>
#include <srslib_framework/platform/Object.hpp>

namespace srs {

class Measurement : public Object
{
public:
    Measurement() :
            Object()
    {}

    virtual ~Measurement()
    {}
};

} // namespace srs

#endif // MEASUREMENT_HPP_
