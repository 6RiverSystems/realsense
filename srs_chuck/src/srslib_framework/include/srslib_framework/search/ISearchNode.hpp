/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <vector>
using namespace std;

namespace srs {

struct ISearchNode
{
    virtual ~ISearchNode() {};

    virtual void freeNode() = 0;

    virtual void getNeighbors(vector<ISearchNode*>& neighbors) = 0;
    virtual int getTotalCost() const = 0;

    virtual bool reachedGoal() const = 0;
};

} // namespace srs
