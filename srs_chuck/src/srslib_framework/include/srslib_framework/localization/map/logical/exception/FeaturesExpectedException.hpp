/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/localization/map/logical/exception/LogicalMapException.hpp>

namespace srs {

class FeaturesExpectedException: public LogicalMapException
{
public:
    FeaturesExpectedException(const LogicalMetadata& metadata) :
        LogicalMapException(metadata,
            "The file file does not contain a features section.")
    {}
};

} // namespace srs
