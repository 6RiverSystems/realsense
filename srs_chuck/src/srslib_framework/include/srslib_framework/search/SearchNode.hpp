/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <vector>
#include <sstream>
using namespace std;

namespace srs {

struct SearchGoal;

struct SearchNode
{
    virtual ~SearchNode()
    {};

    virtual bool equals(SearchNode* const& rhs) const = 0;

    virtual void getExploredNodes(vector<SearchNode*>& exploredNodes) = 0;
    virtual int getLocalCost() const = 0;
    virtual SearchNode* getParent() = 0;
    virtual int getTotalCost() const = 0;
    virtual bool goalReached() const = 0;

    virtual std::size_t hash() const = 0;

    friend ostream& operator<<(ostream& stream, const SearchNode& node)
    {
        return node.toString(stream);
    }

    friend ostream& operator<<(ostream& stream, SearchNode*& node)
    {
        return node->toString(stream);
    }

    virtual void release() = 0;

    virtual void setGoal(SearchGoal* goal) = 0;

    virtual ostream& toString(ostream& stream) const = 0;

    struct EqualTo
    {
        bool operator()(SearchNode* const& lhs, SearchNode* const& rhs) const
        {
            return lhs->equals(rhs);
        }
    };

    struct Hash
    {
        std::size_t operator()(SearchNode* const& node) const
        {
            return node->hash();
        }
    };
};

} // namespace srs
