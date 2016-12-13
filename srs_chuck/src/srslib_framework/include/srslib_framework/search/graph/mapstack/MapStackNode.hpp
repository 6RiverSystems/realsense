/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <sstream>
using namespace std;

#include <srslib_framework/search/SearchGoal.hpp>
#include <srslib_framework/search/SearchNode.hpp>
#include <srslib_framework/search/graph/mapstack/MapStackAction.hpp>

namespace srs {

struct MapStackNode : public SearchNode
{
    struct SearchParameters
    {
        SearchParameters() :
            allowUnknown(false),
            costMapRatio(1.0)
        {}

        bool allowUnknown;
        float costMapRatio;
    };

    static MapStackNode* instanceOfStart(MapStack* stack,
        Grid2d::Position position,
        SearchParameters searchParameters = SearchParameters())
    {
        return instanceOf(stack,
            nullptr, MapStackAction::START,
            position,
            stack->getTotalCost(position,
                searchParameters.allowUnknown, searchParameters.costMapRatio),
            0, nullptr,
            searchParameters);
    }

    static MapStackNode* instanceOf(MapStack* grid,
        MapStackNode* parentNode, MapStackAction::ActionEnum parentAction,
        Grid2d::Position position, int g, int h,
        SearchGoal* goal,
        SearchParameters searchParameters);

    bool equals(SearchNode* const& rhs) const
    {
        return this == rhs ||
            position_ == reinterpret_cast<const MapStackNode*>(rhs)->getPosition();
    }

    int getG() const
    {
        return g_;
    }

    SearchGoal* getGoal()
    {
        return goal_;
    }

    int getH() const
    {
        return h_;
    }

    void getExploredNodes(vector<SearchNode*>& exploredNodes);

    SearchNode* getParent()
    {
        return parentNode_;
    }

    MapStackAction::ActionEnum getParentAction()
    {
        return parentAction_;
    }

    Grid2d::Position getPosition() const
    {
        return position_;
    }

    SearchParameters getSearchParameters() const
    {
        return searchParameters_;
    }

    int getTotalCost() const
    {
        return g_ + h_;
    }

    bool goalReached() const
    {
        return goal_->reached(reinterpret_cast<const MapStackNode*>(this));
    }

    std::size_t hash() const
    {
        return position_.hash();
    }

    void release();

    void setGoal(SearchGoal* goal)
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
    MapStackNode(MapStack* stack,
        MapStackNode* parentNode, MapStackAction::ActionEnum parentAction,
            Grid2d::Position position, int g, int h,
            SearchGoal* goal,
            SearchParameters searchParameters) :
        stack_(stack),
        parentAction_(parentAction),
        parentNode_(parentNode),
        position_(position),
        g_(g),
        goal_(goal),
        h_(h),
        searchParameters_(searchParameters)
    {}

    ~MapStackNode()
    {}

private:
    SearchGoal* goal_;
    MapStack* stack_;
    int g_;

    int h_;

    MapStackAction::ActionEnum parentAction_;
    MapStackNode* parentNode_;
    Grid2d::Position position_;

    SearchParameters searchParameters_;
};

} // namespace srs
