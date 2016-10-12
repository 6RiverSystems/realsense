/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/search/ISearchGoal.hpp>
#include <srslib_framework/search/ISearchNode.hpp>
#include <srslib_framework/search/graph/grid2d/Grid2dPosition.hpp>
#include <srslib_framework/search/graph/grid2d/Grid2dAction.hpp>

namespace srs {

struct Grid2dNode : public ISearchNode
{
    static Grid2dNode* instanceOfStart(Grid2d* graph, Grid2dPosition position, ISearchGoal* goal);

    void freeNode()
    {
        delete this;
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

    bool reachedGoal() const
    {
        return goal_->reached(reinterpret_cast<const Grid2dNode*>(this));
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
