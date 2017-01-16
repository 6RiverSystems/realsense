/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <sstream>
#include <iostream>
#include <iomanip>
#include <vector>
#include <unordered_map>
#include <limits>
#include <functional>
using namespace std;

#include <srslib_framework/math/BasicMath.hpp>

namespace srs {

struct Location
{
    Location(int x = 0, int y = 0) :
        x(x),
        y(y)
    {}

    Location(const Location& other) :
        x(other.x),
        y(other.y)
    {}

    inline size_t hash() const
    {
        return (401 + x) * 401 + y;
    }

    inline friend bool operator==(const Location& lhs, const Location& rhs)
    {
        return (lhs.x == rhs.x) && (lhs.y == rhs.y);
    }

    friend ostream& operator<<(ostream& stream, const Location& location)
    {
        return stream << location.toString();
    }

    string toString() const
    {
        stringstream stream;
        stream << "{" << x << ", " << y << "}";
        return stream.str();
    }

    int x;
    int y;
};

} // namespace srs
