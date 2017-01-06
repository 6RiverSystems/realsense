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

#include <srslib_framework/datastructure/Location.hpp>

namespace srs {

struct Position
{
    Position(int x = 0, int y = 0, int orientation = 0) :
        location(x, y),
        orientation(orientation)
    {}

    Position(Location location, int orientation = 0) :
        location(location),
        orientation(orientation)
    {}

    ~Position()
    {}

    inline std::size_t hash() const
    {
        return ((401 + location.x) * 401 + location.y) * 401 + orientation;
    }

    inline friend bool operator==(const Position& lhs, const Position& rhs)
    {
        return (lhs.location == rhs.location) && (lhs.orientation == rhs.orientation);
    }

    friend ostream& operator<<(ostream& stream, const Position& position)
    {
        return stream << position.toString();
    }

    string toString() const
    {
        stringstream stream;
        stream << "{" << location << ", " << orientation << "}";
        return stream.str();
    }

    Location location;
    int orientation;
};

} // namespace srs
