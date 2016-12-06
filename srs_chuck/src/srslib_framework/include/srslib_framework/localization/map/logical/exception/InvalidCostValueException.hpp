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

class InvalidCostValueException: public LogicalMapException
{
public:
    InvalidCostValueException(const LogicalMetadata& metadata, int value, int minValue, int maxValue) :
        LogicalMapException(metadata,
            "Invalid cost value {actual: " + to_string(value) +
            ", range: [" + to_string(minValue) + ", " + to_string(maxValue) + "]}")
    {}
};

} // namespace srs
