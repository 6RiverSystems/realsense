/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/datastructure/Position.hpp>
#include <srslib_framework/search/SearchGoal.hpp>
#include <srslib_framework/search/graph/grid2d/Grid2dNode.hpp>

namespace srs {

struct Grid2dSingleGoal : public SearchGoal
{
    static Grid2dSingleGoal* instanceOf(Position position)
    {
        return new Grid2dSingleGoal(position);
    }

    int heuristic(const SearchNode* node) const
    {
        return heuristic((reinterpret_cast<const Grid2dNode*>(node))->getPosition());
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
    Grid2dSingleGoal(Position position) :
        goalPosition_(position)
    {}

    ~Grid2dSingleGoal()
    {}

    int heuristic(Position toPosition) const
    {
        return abs(goalPosition_.location.x - toPosition.location.x) +
            abs(goalPosition_.location.y - toPosition.location.y);
    }

    Position goalPosition_;
};

} // namespace srs
