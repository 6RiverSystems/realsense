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
#include <srslib_framework/search/graph/grid2d/Grid2dAction.hpp>

namespace srs {

struct Grid2dNode : public SearchNode
{
    static Grid2dNode* instanceOfStart(Grid2d* grid, Grid2d::Position position)
    {
        return instanceOf(grid,
            nullptr, Grid2dAction::START,
            position,
            grid->getPayload(position), 0, nullptr);
    }

    static Grid2dNode* instanceOf(Grid2d* grid,
        Grid2dNode* parentNode, Grid2dAction::ActionEnum parentAction,
        Grid2d::Position position, int g, int h,
        SearchGoal* goal);

    bool equals(SearchNode* const& rhs) const
    {
        return this == rhs || position_ == reinterpret_cast<const Grid2dNode*>(rhs)->getPosition();
    }

    int getG() const
    {
        return g_;
    }

    int getH() const
    {
        return h_;
    }

    void getNeighbors(vector<SearchNode*>& neighbors);

    SearchNode* getParent()
    {
        return parentNode_;
    }

    Grid2dAction::ActionEnum getParentAction()
    {
        return parentAction_;
    }

    Grid2d::Position getPosition() const
    {
        return position_;
    }

    int getTotalCost() const
    {
        return BasicMath::noOverflowAdd(g_, h_);
    }

    bool goalReached() const
    {
        return goal_->reached(reinterpret_cast<const Grid2dNode*>(this));
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
        return stream << hex << reinterpret_cast<const void*>(this) << dec << " {" <<
            "p: " << hex << reinterpret_cast<const void*>(parentNode_) << dec <<
            ", a: " << parentAction_ <<
            ", pos: " << position_ <<
            ", g: " << g_ <<
            ", h: " << h_ <<
            ", goal: " << *goal_ <<
            "}";
    }

protected:
    Grid2dNode(Grid2d* graph,
            Grid2dNode* parentNode, Grid2dAction::ActionEnum parentAction,
            Grid2d::Position position, int g, int h,
            SearchGoal* goal) :
        graph_(graph),
        parentAction_(parentAction),
        parentNode_(parentNode),
        position_(position),
        g_(g),
        goal_(goal),
        h_(h)
    {}

    ~Grid2dNode()
    {}

private:
    SearchGoal* goal_;
    Grid2d* graph_;
    int g_;

    int h_;

    Grid2dAction::ActionEnum parentAction_;
    Grid2dNode* parentNode_;
    Grid2d::Position position_;
};

} // namespace srs
