/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <array>
#include <vector>
#include <limits>
#include <unordered_set>
using namespace std;

#include <srslib_framework/datastructure/queue/MappedPriorityQueue.hpp>
#include <srslib_framework/search/ISearchNode.hpp>

namespace srs {

class AStar
{
public:
    AStar() :
        lastNode_(nullptr),
        startNode_(nullptr)
    {}

    ~AStar()
    {}

    void clear();

    unsigned int getClosedNodeCount() const
    {
        return closed_.size();
    }

    unsigned int getOpenNodeCount() const
    {
        return open_.size();
    }

    ISearchNode* getSolution() const
    {
        return lastNode_;
    }

    bool hasSolution() const
    {
        return lastNode_;
    }

    bool search(ISearchNode* start, ISearchGoal* goal);

private:
    typedef unordered_set<ISearchNode*,
        ISearchNode::Hash, ISearchNode::EqualTo> ClosedSetType;
    typedef MappedPriorityQueue<ISearchNode*, unsigned int,
        ISearchNode::Hash, ISearchNode::EqualTo> OpenSetType;

    void pushNodes(vector<ISearchNode*>& nodes);

    ClosedSetType closed_;

    ISearchNode* startNode_;
    ISearchNode* lastNode_;

    OpenSetType open_;
};

} // namespace srs
