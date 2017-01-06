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
#include <srslib_framework/datastructure/Position.hpp>
#include <srslib_framework/math/BasicMath.hpp>

namespace srs {

class BaseGrid2d
{
public:
    using BaseType = unsigned char;

    enum {
        ORIENTATION_NORTH = 90,
        ORIENTATION_EAST = 0,
        ORIENTATION_SOUTH = 270,
        ORIENTATION_WEST = 180
    };

    BaseGrid2d(unsigned int size) :
        width_(size),
        height_(size)
    {}

    BaseGrid2d(unsigned int width, unsigned int height) :
        width_(width),
        height_(height)
    {}

    ~BaseGrid2d()
    {}

    unsigned int getHeight() const
    {
        return height_;
    }

    bool getNeighbor(const Position& position, Position& result) const
    {
        switch (position.orientation)
        {
            case ORIENTATION_EAST:
                result = Position(position.location.x + 1, position.location.y, position.orientation);
                break;

            case ORIENTATION_NORTH:
                result = Position(position.location.x, position.location.y + 1, position.orientation);
                break;

            case ORIENTATION_WEST:
                result = Position(position.location.x - 1, position.location.y, position.orientation);
                break;

            case ORIENTATION_SOUTH:
                result = Position(position.location.x, position.location.y - 1, position.orientation);
                break;

            default:
                return false;
        }

        return isWithinBounds(result);
    }

    unsigned int getWidth() const
    {
        return width_;
    }

    inline bool isWithinBounds(const Location& location) const
    {
        return (0 <= location.x && location.x < width_) &&
            (0 <= location.y && location.y < height_);
    }

    inline bool isWithinBounds(const Position& position) const
    {
        return (0 <= position.location.x && position.location.x < width_) &&
            (0 <= position.location.y && position.location.y < height_);
    }

private:
    unsigned int height_;

    unsigned int width_;
};

} // namespace srs
