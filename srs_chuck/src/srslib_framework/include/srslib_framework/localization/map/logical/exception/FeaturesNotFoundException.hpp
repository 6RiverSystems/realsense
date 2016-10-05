/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/exception/SrsRuntimeErrorException.hpp>

namespace srs {

class FeaturesNotFoundException: public SrsRuntimeErrorException
{
public:
    FeaturesNotFoundException(const string& filename) :
        SrsRuntimeErrorException("The geojson file does not contain a features section [" +
            filename + "]")
    {}
};

} // namespace srs
