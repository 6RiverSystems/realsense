/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/search/ISearchNode.hpp>

namespace srs {

struct ISearchGoal
{
    virtual ~ISearchGoal() {};

    virtual int heuristic(const ISearchNode* node) const = 0;

    friend ostream& operator<<(ostream& stream, const ISearchGoal& goal)
    {
        return goal.toString(stream);
    }

    virtual bool reached(const ISearchNode* node) const = 0;

    virtual ostream& toString(ostream& stream) const = 0;
};

} // namespace srs
