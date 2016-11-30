/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <list>
#include <sstream>
#include <iomanip>
using namespace std;

namespace srs {

template<typename SOLUTION_ITEM>
class Solution : public list<SOLUTION_ITEM>
{
public:
    Solution()
    {}

    Solution(SOLUTION_ITEM firstNode)
    {
        list<SOLUTION_ITEM>::push_back(firstNode);
    }

    void append(Solution<SOLUTION_ITEM>* other)
    {
        list<SOLUTION_ITEM>::insert(list<SOLUTION_ITEM>::end(), other->begin(), other->end());
    }

    SOLUTION_ITEM getGoal() const
    {
        return list<SOLUTION_ITEM>::back();
    }

    SOLUTION_ITEM getStart() const
    {
        return list<SOLUTION_ITEM>::front();
    }

    friend ostream& operator<<(ostream& stream, const Solution& solution)
    {
        return stream << solution.toString();
    }

    string toString() const
    {
        stringstream stream;

        int counter = 0;

        stream << "{" << endl;
        for (auto node : *this)
        {
            stream << setw(4) << counter++ << ": " << node << endl;
        }

        stream << "}";

        return stream.str();
    }
};

} // namespace srs
