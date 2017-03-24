/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/exception/SrsRuntimeErrorException.hpp>

namespace srs {

class FailedToOpenFileException: public SrsRuntimeErrorException
{
public:
    FailedToOpenFileException(const string& filename) :
        SrsRuntimeErrorException("Failed to open file [" + filename + "]")
    {}
};

} // namespace srs
