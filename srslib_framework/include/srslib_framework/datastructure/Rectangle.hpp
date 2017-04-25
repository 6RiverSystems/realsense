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

struct Rectangle
{
    Rectangle(unsigned int x0 = 0, unsigned int y0 = 0, unsigned int x1 = 0, unsigned int y1 = 0) :
        x0(x0),
        y0(y0),
        x1(x1),
        y1(y1)
    {}

    Rectangle(const Rectangle& other) :
        x0(other.x0),
        y0(other.y0),
        x1(other.x1),
        y1(other.y1)
    {}

    bool isIn(unsigned int x, unsigned int y)
    {
        return x >= x0 &&  x <= x1 && y >= y0 && y <= y1;
    }

    inline friend bool operator==(const Rectangle& lhs, const Rectangle& rhs)
    {
        return (lhs.x0 == rhs.x0) && (lhs.y0 == rhs.y0) &&
            (lhs.x1 == rhs.x1) && (lhs.y1 == rhs.y1);
    }

    friend ostream& operator<<(ostream& stream, const Rectangle& rectangle)
    {
        return stream << rectangle.toString();
    }

    string toString() const
    {
        stringstream stream;
        stream << "{" << x0 << ", " << y0 << ", " << x1 << ", " << y1 << "}";
        return stream.str();
    }

    unsigned int x0;
    unsigned int y0;

    unsigned int x1;
    unsigned int y1;
};

} // namespace srs
