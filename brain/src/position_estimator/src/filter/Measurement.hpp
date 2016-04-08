/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef MEASUREMENT_HPP_
#define MEASUREMENT_HPP_

#include <platform/Object.hpp>

namespace srs {

/**
 * Interface for the sensor frame queue.
 */
struct Measurement : public Object
{
    Measurement()
    {}

    virtual ~Measurement()
    {}
};

} // namespace srs

#endif // MEASUREMENT_HPP_
