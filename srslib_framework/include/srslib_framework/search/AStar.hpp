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
#include <srslib_framework/search/SearchGoal.hpp>

namespace srs {

template <typename NODETYPE>
class AStar
{
public:
    /*
     * Sets the expected number of nodes to accommodate at least count elements without
     * exceeding maximum load factor and rehashes the closed list. Since, at the moment,
     * A* performs a search primarily on maps, the value has been estimated on the
     * average path search.
     */
    static constexpr int CLOSED_HASH_RESERVE = 400000;

    AStar();
    virtual ~AStar();

    void clear();

    unsigned int getClosedNodesCount() const
    {
        return closedSet_.size();
    }

    unsigned int getOpenNodesCount() const
    {
        return openQueue_.size();
    }

    void getPlan(Plan<NODETYPE>& plan);

    #if DIAGNOSTICS_ASTAR

        unsigned int getFoundInClosed() const
        {
            return counterFoundInClosed_;
        }

        unsigned int getInserted() const
        {
            return counterInserted_;
        }

        unsigned int getPruned() const
        {
            return counterPruned_;
        }

        unsigned int getReplaced() const
        {
            return counterReplaced_;
        }

    #endif

    bool hasSolution() const
    {
        return lastNode_;
    }

    bool search(NODETYPE* start, SearchGoal<NODETYPE>* goal);

protected:
    using ClosedSetType = unordered_set<NODETYPE*, typename NODETYPE::Hash, typename NODETYPE::EqualTo>;
    using OpenSetType = MappedPriorityQueue<NODETYPE*, unsigned int, typename NODETYPE::Hash, typename NODETYPE::EqualTo>;

    void pushNodes(vector<NODETYPE*>& nodes);

    virtual void getExploredNodes(NODETYPE*, std::vector<NODETYPE*>& ) = 0;
    virtual void releaseNode(NODETYPE*) = 0;

    #if DIAGNOSTICS_ASTAR
        unsigned int counterFoundInClosed_  { 0 };
        unsigned int counterInserted_       { 0 };
        unsigned int counterPruned_         { 0 };
        unsigned int counterReplaced_       { 0 };
    #endif

    NODETYPE* lastNode_                     { nullptr };

    NODETYPE* startNode_                    { nullptr };

    ClosedSetType closedSet_                { };

    OpenSetType openQueue_                  { };
};

} // namespace srs
