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
#include <srslib_framework/search/Plan.hpp>
#include <srslib_framework/search/SearchNode.hpp>

namespace srs {

class AStar
{
public:
    struct ConfigParameters
    {
        ConfigParameters() :
            useYield(true),
            yieldFrequency(2000)
        {}

        bool useYield;
        unsigned int yieldFrequency;
    };

    AStar() :
        lastNode_(nullptr),
        startNode_(nullptr),
        yieldCounter_(0)
    {}

    ~AStar()
    {
        clear();
    }

    void clear();

    unsigned int getClosedNodesCount() const
    {
        return closedSet_.size();
    }

    unsigned int getOpenNodesCount() const
    {
        return openQueue_.size();
    }

    void getPlan(Plan& plan);

    bool hasSolution() const
    {
        return lastNode_;
    }

    bool search(SearchNode* start, SearchGoal* goal,
        ConfigParameters parameters = ConfigParameters());

private:
    using ClosedSetType = unordered_set<SearchNode*, SearchNode::Hash, SearchNode::EqualTo>;
    using OpenSetType = MappedPriorityQueue<SearchNode*, unsigned int,
        SearchNode::Hash, SearchNode::EqualTo>;

    void pushNodes(vector<SearchNode*>& nodes);

    ClosedSetType closedSet_;

    SearchNode* lastNode_;

    OpenSetType openQueue_;

    SearchNode* startNode_;

    unsigned int yieldCounter_;
};

} // namespace srs
