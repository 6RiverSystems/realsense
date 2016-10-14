/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <sstream>
using namespace std;

#include <srslib_framework/search/ISearchGoal.hpp>
#include <srslib_framework/search/ISearchNode.hpp>
#include <srslib_framework/search/graph/grid2d/Grid2dPosition.hpp>
#include <srslib_framework/search/graph/grid2d/Grid2dAction.hpp>

namespace srs {

struct Grid2dNode : public ISearchNode
{
    static Grid2dNode* instanceOfStart(Grid2d* graph, Grid2dPosition position)
    {
        return new Grid2dNode(graph,
            nullptr, Grid2dAction::START,
            position,
            0, 0, nullptr);
    }

    bool equals(ISearchNode* const& rhs) const
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

    void getNeighbors(vector<ISearchNode*>& neighbors);

    Grid2dPosition getPosition() const
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

    void release()
    {
        delete this;
    }

    void setGoal(ISearchGoal* goal)
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

private:
    Grid2dNode(Grid2d* graph,
            Grid2dNode* parentNode, Grid2dAction::ActionEnum parentAction,
            Grid2dPosition position, int g, int h,
            ISearchGoal* goal) :
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

    ISearchGoal* goal_;
    Grid2d* graph_;
    int g_;

    int h_;

    Grid2dAction::ActionEnum parentAction_;
    Grid2dNode* parentNode_;
    Grid2dPosition position_;
};

} // namespace srs
