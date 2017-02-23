/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/exception/SrsLogicErrorException.hpp>

namespace srs {

class IncompatibleLanguageVersionException: public LogicalMapException
{
public:
    IncompatibleLanguageVersionException(const LogicalMetadata& metadata,
        const string& expected, const string& actual) :
            LogicalMapException(metadata,
                "Incompatible logical map language version. Expected: " +
                expected + ", but found: " + actual)
    {}
};

} // namespace srs
