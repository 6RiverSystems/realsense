/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

namespace srs {

template <typename SEARCHNODE>
struct SearchGoal
{
    virtual ~SearchGoal()
    {};

    virtual int heuristic(const SEARCHNODE* node) const = 0;

    friend ostream& operator<<(ostream& stream, const SearchGoal<SEARCHNODE>& goal)
    {
        return goal.toString(stream);
    }

    virtual bool reached(const SEARCHNODE* node) const = 0;

    virtual void release() = 0;

    virtual ostream& toString(ostream& stream) const = 0;
};

} // namespace srs
