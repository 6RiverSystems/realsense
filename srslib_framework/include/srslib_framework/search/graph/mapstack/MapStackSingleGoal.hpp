/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/datastructure/Position.hpp>
#include <srslib_framework/search/SearchGoal.hpp>
#include <srslib_framework/search/graph/mapstack/MapStackNode.hpp>

namespace srs {

struct MapStackSingleGoal : public SearchGoal<MapStackNode>
{
    static MapStackSingleGoal* instanceOf(Position position)
    {
        return new MapStackSingleGoal(position);
    }

    virtual int heuristic(const MapStackNode* node) const final
    {
        return heuristic(node->getPosition());
    }

    virtual bool reached(const MapStackNode* node) const final
    {
        return goalPosition_ == node->getPosition();
    }

    virtual void release() final
    {
        delete this;
    }

    virtual ostream& toString(ostream& stream) const final
    {
        return stream << hex << reinterpret_cast<const void*>(this) << dec << " {"
            "p: " << goalPosition_ <<
            "}";
    }

private:
    MapStackSingleGoal(const Position& position) :
        goalPosition_(position)
    {}

    ~MapStackSingleGoal()
    {}

    int heuristic(const Position& toPosition) const
    {
        return abs(goalPosition_.location.x - toPosition.location.x) +
            abs(goalPosition_.location.y - toPosition.location.y);
    }

    Position goalPosition_;
};

} // namespace srs
