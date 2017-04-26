/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/localization/map/logical/exception/LogicalMapException.hpp>

namespace srs {

class UnexpectedValueException: public LogicalMapException
{
public:
    UnexpectedValueException(const LogicalMetadata& metadata, const string& value) :
        LogicalMapException(metadata,
            "Unexpected value {" + value + "}")
    {}
};

} // namespace srs
