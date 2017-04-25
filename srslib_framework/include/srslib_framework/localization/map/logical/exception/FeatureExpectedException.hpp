/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/localization/map/logical/exception/LogicalMapException.hpp>

namespace srs {

class FeatureExpectedException: public LogicalMapException
{
public:
    FeatureExpectedException(const LogicalMetadata& metadata, const string& feature) :
        LogicalMapException(metadata,
            "The file does not contain the required feature {" + feature + "}")
    {}
};

} // namespace srs
