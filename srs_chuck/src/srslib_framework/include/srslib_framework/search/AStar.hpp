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

template<typename HASH, typename EQUAL_TO>
class AStar
{
public:
    AStar() :
        lastNode_(nullptr)
    {}

    ~AStar()
    {}

    void clear();

    unsigned int getClosedNodeCount()
    {
        return closed_.size();
    }

    unsigned int getOpenNodeCount()
    {
        return open_.size();
    }

    bool search(ISearchNode* startNode);

private:
    void pushNodes(vector<ISearchNode*>& nodes);

    unordered_set<ISearchNode*> closed_;

    ISearchNode* lastNode_;

    MappedPriorityQueue<ISearchNode*, unsigned int> open_;
};

} // namespace srs

#include <search/AStar.cpp>
