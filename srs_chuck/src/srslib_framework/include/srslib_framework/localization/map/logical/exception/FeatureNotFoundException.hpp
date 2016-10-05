/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/exception/SrsRuntimeErrorException.hpp>

namespace srs {

class FeatureNotFoundException: public SrsRuntimeErrorException
{
public:
    FeatureNotFoundException(const string& filename, const string& feature) :
        SrsRuntimeErrorException("The geojson file does not contain the required feature [" +
            filename + ", " + feature + "]")
    {}
};

} // namespace srs
