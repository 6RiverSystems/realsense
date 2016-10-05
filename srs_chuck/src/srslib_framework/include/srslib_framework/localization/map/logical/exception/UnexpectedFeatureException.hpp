/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/exception/SrsRuntimeErrorException.hpp>

namespace srs {

class UnexpectedFeatureException: public SrsRuntimeErrorException
{
public:
    UnexpectedFeatureException(const string& filename, const string& feature) :
        SrsRuntimeErrorException("The feature was not expected [" +
            filename + ", " + feature + "]")
    {}
};

} // namespace srs
