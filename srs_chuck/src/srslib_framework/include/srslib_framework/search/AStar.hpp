/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <array>
#include <list>
#include <limits>
#include <unordered_set>
using namespace std;

#include <srslib_framework/datastructure/queue/MappedPriorityQueue.hpp>
#include <srslib_framework/search/SearchNode.hpp>

namespace srs {

class AStar
{
public:
    typedef list<SearchNode*> SolutionType;

    AStar() :
        lastNode_(nullptr),
        startNode_(nullptr)
    {}

    ~AStar()
    {
        clear();
    }

    void clear();

    unsigned int getClosedNodeCount() const
    {
        return closed_.size();
    }

    unsigned int getOpenNodeCount() const
    {
        return open_.size();
    }

    void getSolution(SolutionType& solution);

    bool hasSolution() const
    {
        return lastNode_;
    }

    bool search(SearchNode* start, SearchGoal* goal);

private:
    typedef unordered_set<SearchNode*,
        SearchNode::Hash, SearchNode::EqualTo> ClosedSetType;
    typedef MappedPriorityQueue<SearchNode*, unsigned int,
        SearchNode::Hash, SearchNode::EqualTo> OpenSetType;

    void pushNodes(vector<SearchNode*>& nodes);

    ClosedSetType closed_;

    SearchNode* startNode_;
    SearchNode* lastNode_;

    OpenSetType open_;
};

} // namespace srs
