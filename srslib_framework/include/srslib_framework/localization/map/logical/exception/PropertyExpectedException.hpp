/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/localization/map/logical/exception/LogicalMapException.hpp>

namespace srs {

class PropertyExpectedException: public LogicalMapException
{
public:
    PropertyExpectedException(const LogicalMetadata& metadata,
        const string& feature, const string& property) :
        LogicalMapException(metadata,
            "The entity does not contain the required property {" + feature + ", " + property + "}")
    {}
};

} // namespace srs
