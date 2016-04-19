/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef MEASUREMENT_HPP_
#define MEASUREMENT_HPP_

#include <opencv2/opencv.hpp>

#include <platform/Object.hpp>
#include <platform/Ocv2Base.hpp>

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
