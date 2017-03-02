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
        ORIENTATION_NORTH_EAST = 45,
        ORIENTATION_EAST = 0,
        ORIENTATION_SOUTH_EAST = 315,
        ORIENTATION_SOUTH = 270,
        ORIENTATION_SOUTH_WEST = 225,
        ORIENTATION_WEST = 180,
        ORIENTATION_NORTH_WEST = 135,
    };

    BaseGrid2d(unsigned int size) :
        width_(size),
        height_(size)
    {}

    BaseGrid2d(unsigned int width, unsigned int height) :
        width_(width),
        height_(height)
    {}

    virtual ~BaseGrid2d()
    {}

    unsigned int getHeight() const
    {
        return height_;
    }

    virtual BaseType getMinPayloadValue() = 0;
    virtual BaseType getMaxPayloadValue() = 0;

    bool getNeighbor(const Position& position, Position& result) const
    {
        switch (position.orientation)
        {
            case ORIENTATION_NORTH:
                result = Position(position.location.x, position.location.y + 1,
                    position.orientation);
                break;

            case ORIENTATION_NORTH_EAST:
                result = Position(position.location.x + 1, position.location.y + 1,
                    position.orientation);
                break;

            case ORIENTATION_EAST:
                result = Position(position.location.x + 1, position.location.y,
                    position.orientation);
                break;

            case ORIENTATION_SOUTH_EAST:
                result = Position(position.location.x + 1, position.location.y - 1,
                    position.orientation);
                break;

            case ORIENTATION_SOUTH:
                result = Position(position.location.x, position.location.y - 1,
                    position.orientation);
                break;

            case ORIENTATION_SOUTH_WEST:
                result = Position(position.location.x - 1, position.location.y - 1,
                    position.orientation);
                break;

            case ORIENTATION_WEST:
                result = Position(position.location.x - 1, position.location.y,
                    position.orientation);
                break;

            case ORIENTATION_NORTH_WEST:
                result = Position(position.location.x - 1, position.location.y + 1,
                    position.orientation);
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

    virtual void payloadMax(const Location& location, BaseType otherPayload) = 0;
    virtual void payloadSet(const Location& location, BaseType newPayload) = 0;

private:
    unsigned int height_;

    unsigned int width_;
};

} // namespace srs
