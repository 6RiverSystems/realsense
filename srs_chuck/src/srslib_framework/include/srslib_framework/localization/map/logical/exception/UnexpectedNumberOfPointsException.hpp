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

class UnexpectedNumberOfPointsException: public LogicalMapException
{
public:
    UnexpectedNumberOfPointsException(const LogicalMetadata& metadata,
        int expected, int actual) :
            LogicalMapException(metadata,
                "The geometry does not define enough points [expected: " +
                to_string(expected) + ", actual: " + to_string(actual) + "]")
    {}

    UnexpectedNumberOfPointsException(const LogicalMetadata& metadata,
        int fromExpected, int toExpected, int actual) :
            LogicalMapException(metadata,
                "The geometry does not define enough points [expected: <" +
                to_string(fromExpected) + " - " + to_string(toExpected) + ">" +
                ", actual: " + to_string(actual) + "]")
    {}
};

} // namespace srs
