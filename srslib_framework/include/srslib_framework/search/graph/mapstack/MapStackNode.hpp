/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <sstream>
using namespace std;

#include <srslib_framework/datastructure/Position.hpp>
#include <srslib_framework/search/SearchGoal.hpp>
#include <srslib_framework/search/SearchNode.hpp>
#include <srslib_framework/search/graph/mapstack/MapStackAction.hpp>

namespace srs {

struct MapStackSearchParameters
{
    MapStackSearchParameters() :
        allowUnknown(false),
        costMapRatio(1.0)
    {}

    bool allowUnknown;
    float costMapRatio;
};

struct MapStackNode
{
    struct EqualTo
    {
        constexpr bool operator()(const MapStackNode* lhs, const MapStackNode* rhs) const
        {
            return lhs == rhs ||
                lhs->position_ == rhs->position_;
        }
    };

    struct Hash
    {
        size_t operator()(const MapStackNode*node) const
        {
            return node->position_.hash();
        }
    };

    static MapStackNode* instanceOfStart(MapStack*stack, Position position, const MapStackSearchParameters& searchParameters)
    {
        return instanceOf(
            nullptr, MapStackAction::START,
            position,
            stack->getTotalCost(position,
                searchParameters.allowUnknown, searchParameters.costMapRatio),
            0, nullptr);
    }

    static MapStackNode* instanceOf(
        MapStackNode* parentNode, MapStackAction::ActionEnum parentAction,
        Position position, int g, int h,
        SearchGoal<MapStackNode>* goal);

    SearchGoal<MapStackNode>* getGoal() const
    {
        return goal_;
    }

    int getH() const
    {
        return h_;
    }

    int getLocalCost() const
    {
        return g_;
    }

    MapStackNode* getParent() const
    {
        return parentNode_;
    }

    MapStackAction::ActionEnum getParentAction() const
    {
        return parentAction_;
    }

    const Position& getPosition() const
    {
        return position_;
    }

    int getTotalCost() const
    {
        return g_ + h_;
    }

    bool goalReached() const
    {
        return goal_->reached(this);
    }

    void release();

    void setGoal(SearchGoal<MapStackNode>* goal)
    {
        if (goal)
        {
            goal_ = goal;

            // Calculate the total cost of the node, which,
            // for the start node, is the value of the
            // heuristic function to the goal
            h_ = goal_->heuristic(this);
        }
    }

    ostream& toString(ostream& stream) const
    {
        stream << hex << reinterpret_cast<const void*>(this) << dec << " {" <<
            "p: " << hex << reinterpret_cast<const void*>(parentNode_) << dec <<
            ", a: " << parentAction_ <<
            ", pos: " << position_ <<
            ", g: " << g_ <<
            ", h: " << h_ <<
            ", goal: ";

            if (goal_)
            {
                stream << *goal_;
            }
            else
            {
                stream << "nullptr";
            }

            return stream << "}";
    }

protected:
    MapStackNode(
        MapStackNode* parentNode, MapStackAction::ActionEnum parentAction,
            Position position, int g, int h,
            SearchGoal<MapStackNode>* goal) :
        position_(position),
        g_(g),
        h_(h),
        parentNode_(parentNode),
        parentAction_(parentAction),
        goal_(goal)
    {}

    ~MapStackNode()
    {}

private:

    Position                    position_       { };
    int                         g_              { 0 };
    int                         h_              { 0 };
    MapStackNode*               parentNode_     { nullptr };
    MapStackAction::ActionEnum  parentAction_   { MapStackAction::NONE };
    SearchGoal<MapStackNode>*   goal_           { nullptr };
};

} // namespace srs
