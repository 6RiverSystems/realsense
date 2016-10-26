/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/localization/map/logical/exception/LogicalMapException.hpp>

namespace srs {

class CostExpectedException: public LogicalMapException
{
public:
    CostExpectedException(const LogicalMetadata& metadata) :
        LogicalMapException(metadata,
            "Expected value of type double or 'max'")
    {}
};

} // namespace srs
