/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/localization/map/logical/exception/LogicalMapException.hpp>

namespace srs {

class GeoJsonTypeUnsupportedException: public LogicalMapException
{
public:
    GeoJsonTypeUnsupportedException(const LogicalMetadata& metadata, const string& type) :
        LogicalMapException(metadata,
            "GeoJson type not supported [" + type + "]")
    {}
};

} // namespace srs
