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

struct ISearchGoal;

struct ISearchNode
{
    virtual ~ISearchNode() {};

    virtual bool equals(ISearchNode* const& rhs) const = 0;

    virtual void getNeighbors(vector<ISearchNode*>& neighbors) = 0;
    virtual int getTotalCost() const = 0;
    virtual bool goalReached() const = 0;

    virtual std::size_t hash() const = 0;

    friend ostream& operator<<(ostream& stream, const ISearchNode& node)
    {
        return node.toString(stream);
    }

    friend ostream& operator<<(ostream& stream, ISearchNode*& node)
    {
        return node->toString(stream);
    }

    virtual void release() = 0;

    virtual void setGoal(ISearchGoal* goal) = 0;

    virtual ostream& toString(ostream& stream) const = 0;

    struct EqualTo
    {
        bool operator()(ISearchNode* const& lhs, ISearchNode* const& rhs) const
        {
            return lhs->equals(rhs);
        }
    };

    struct Hash
    {
        std::size_t operator()(ISearchNode* const& node) const
        {
            return node->hash();
        }
    };
};

} // namespace srs
