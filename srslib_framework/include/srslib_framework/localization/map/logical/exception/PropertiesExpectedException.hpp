/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/localization/map/logical/exception/LogicalMapException.hpp>

namespace srs {

class PropertiesExpectedException: public LogicalMapException
{
public:
    PropertiesExpectedException(const LogicalMetadata& metadata, const string& id) :
        LogicalMapException(metadata,
            "The feature does not contain the required properties field {" + id + "}")
    {}
};

} // namespace srs
