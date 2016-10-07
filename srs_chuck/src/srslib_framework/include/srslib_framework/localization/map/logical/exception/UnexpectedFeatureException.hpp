/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/localization/map/logical/exception/LogicalMapException.hpp>

namespace srs {

class UnexpectedFeatureException: public LogicalMapException
{
public:
    UnexpectedFeatureException(const LogicalMetadata& metadata, const string& feature) :
        LogicalMapException(metadata,
            "The feature was not expected [" + feature + "]")
    {}
};

} // namespace srs
