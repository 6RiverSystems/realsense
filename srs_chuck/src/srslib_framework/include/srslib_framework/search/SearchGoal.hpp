/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/search/SearchNode.hpp>

namespace srs {

struct SearchGoal
{
    virtual ~SearchGoal()
    {};

    virtual int heuristic(const SearchNode* node) const = 0;

    friend ostream& operator<<(ostream& stream, const SearchGoal& goal)
    {
        return goal.toString(stream);
    }

    virtual bool reached(const SearchNode* node) const = 0;

    virtual void release() = 0;

    virtual ostream& toString(ostream& stream) const = 0;
};

} // namespace srs
