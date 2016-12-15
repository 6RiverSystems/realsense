/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/search/SearchGoal.hpp>
#include <srslib_framework/search/graph/mapstack/MapStackNode.hpp>

namespace srs {

struct MapStackSingleGoal : public SearchGoal
{
    static MapStackSingleGoal* instanceOf(Grid2d::Position position)
    {
        return new MapStackSingleGoal(position);
    }

    int heuristic(const SearchNode* node) const
    {
        return heuristic((reinterpret_cast<const MapStackNode*>(node))->getPosition());
    }

    bool reached(const SearchNode* node) const
    {
        return goalPosition_ == reinterpret_cast<const MapStackNode*>(node)->getPosition();
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
    MapStackSingleGoal(Grid2d::Position position) :
        goalPosition_(position)
    {}

    ~MapStackSingleGoal()
    {}

    int heuristic(Grid2d::Position toPosition) const
    {
        return abs(goalPosition_.x - toPosition.x) + abs(goalPosition_.y - toPosition.y);
    }

    Grid2d::Position goalPosition_;
};

} // namespace srs
