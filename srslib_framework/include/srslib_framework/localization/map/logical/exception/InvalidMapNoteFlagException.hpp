/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
using namespace std;

#include <srslib_framework/localization/map/logical/exception/LogicalMapException.hpp>

namespace srs {

class InvalidMapNoteFlagException: public LogicalMapException
{
public:
    InvalidMapNoteFlagException(const LogicalMetadata& metadata, string flag, string value) :
        LogicalMapException(metadata,
            "Invalid Map Note flag {" + flag + ": " + value + "}")
    {}
};

} // namespace srs
