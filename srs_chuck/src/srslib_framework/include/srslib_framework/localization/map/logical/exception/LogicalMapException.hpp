/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/exception/SrsRuntimeErrorException.hpp>

namespace srs {

class LogicalMapException: public SrsRuntimeErrorException
{
public:
    LogicalMapException(const LogicalMetadata& metadata, const string& what) :
        SrsRuntimeErrorException(metadata.logicalFilename + ": " + what)
    {}
};

} // namespace srs
