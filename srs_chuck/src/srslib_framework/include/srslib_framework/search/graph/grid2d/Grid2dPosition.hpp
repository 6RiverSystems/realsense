/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <functional>
using namespace std;

#include <srslib_framework/datastructure/graph/grid2d/Grid2d.hpp>

namespace srs {

struct Grid2dPosition
{
    Grid2dPosition(Grid2d::Location location = Grid2d::Location(), int orientation = 0) :
        location(location),
        orientation(orientation)
    {}

    ~Grid2dPosition()
    {}

    friend bool operator==(const Grid2dPosition& lhs, const Grid2dPosition& rhs)
    {
        return (lhs.location == rhs.location) && (lhs.orientation == rhs.orientation);
    }

    friend ostream& operator<<(ostream& stream, const Grid2dPosition& position)
    {
        return stream << "{" << position.location << ", " << position.orientation << "}";
    }

    Grid2d::Location location;
    int orientation;
};

} // namespace srs
