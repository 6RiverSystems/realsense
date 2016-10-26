/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/exception/SrsRuntimeErrorException.hpp>

namespace srs {

class OutOfRangeException: public SrsRuntimeErrorException
{
public:
    OutOfRangeException(const Grid2d::Location& location, const Grid2d::Location max) :
        SrsRuntimeErrorException("Location out of range [" + location.toString() +
            ", limits: " + max.toString() + "]")
    {}
};

} // namespace srs
