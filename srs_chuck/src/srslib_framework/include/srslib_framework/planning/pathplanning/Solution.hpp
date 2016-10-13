/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <iomanip>

namespace srs {

template<typename SOLUTION_ITEM>
class Solution : public vector<SOLUTION_ITEM>
{
public:
    Solution()
    {}

    Solution(SOLUTION_ITEM firstNode)
    {
        vector<SOLUTION_ITEM>::push_back(firstNode);
    }

    void append(Solution<SOLUTION_ITEM>* other)
    {
        vector<SOLUTION_ITEM>::insert(vector<SOLUTION_ITEM>::end(), other->begin(), other->end());
    }

    SOLUTION_ITEM getGoal() const
    {
        return *(vector<SOLUTION_ITEM>::end() - 1);
    }

    SOLUTION_ITEM getStart() const
    {
        return *vector<SOLUTION_ITEM>::begin();
    }

    friend ostream& operator<<(ostream& stream, const Solution& solution)
    {
        int counter = 0;

        stream << "{" << endl;
        for (auto node : solution)
        {
            stream << setw(4) << counter++ << ": " << node << endl;
        }

        return stream << "}";
    }
};

} // namespace srs
