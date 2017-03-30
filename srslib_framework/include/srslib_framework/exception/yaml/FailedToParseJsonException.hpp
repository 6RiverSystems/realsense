/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/exception/SrsLogicErrorException.hpp>

namespace srs {

class FailedToParseJsonException: public SrsLogicErrorException
{
public:
    FailedToParseJsonException(const string& json) :
        SrsLogicErrorException("Failed to parse Json string [" + json + "]")
    {}
};

} // namespace srs
