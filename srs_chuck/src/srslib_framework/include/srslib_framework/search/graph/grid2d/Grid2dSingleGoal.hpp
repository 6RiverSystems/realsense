/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/search/SearchGoal.hpp>
#include <srslib_framework/search/graph/grid2d/Grid2dNode.hpp>
#include <srslib_framework/search/graph/grid2d/Grid2dPosition.hpp>

namespace srs {

struct Grid2dSingleGoal : public SearchGoal
{
    static Grid2dSingleGoal* instanceOf(Grid2dPosition position)
    {
        return new Grid2dSingleGoal(position);
    }

    int heuristic(const SearchNode* node) const
    {
        return heuristic((reinterpret_cast<const Grid2dNode*>(node))->getPosition().location);
    }

    bool reached(const SearchNode* node) const
    {
        return goalPosition_ == reinterpret_cast<const Grid2dNode*>(node)->getPosition();
    }

    void release()
    {
        delete this;
    }

    ostream& toString(ostream& stream) const
    {
        return stream << hex << reinterpret_cast<const void*>(this) << dec << " {"
            "p: " << goalPosition_ <<
            "}";
    }

private:
    Grid2dSingleGoal(Grid2dPosition position) :
        goalPosition_(position)
    {}

    ~Grid2dSingleGoal()
    {}

    int heuristic(Grid2d::Location toLocation) const
    {
        return abs(goalPosition_.location.x - toLocation.x) +
            abs(goalPosition_.location.y - toLocation.y);
    }

    Grid2dPosition goalPosition_;
};

} // namespace srs
