/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef SOLUTION_HPP_
#define SOLUTION_HPP_

#include <iomanip>

namespace srs {

template<typename SOLUTION_ITEM>
class Solution : public vector<SOLUTION_ITEM>
{
public:
    typedef SOLUTION_ITEM NodeType;

    Solution()
    {}

    Solution(NodeType firstNode)
    {
        vector<NodeType>::push_back(firstNode);
    }

    NodeType getStart()
    {
        return *vector<NodeType>::begin();
    }

    NodeType getGoal()
    {
        return *(vector<NodeType>::end() - 1);
    }

    friend ostream& operator<<(ostream& stream, const Solution& solution)
    {
        int counter = 0;

        stream << "Solution {" << endl;
        for (auto node : solution)
        {
            stream << setw(4) << counter++ << ": " << node << endl;
        }

        return stream << "}";
    }
};

} // namespace srs

#endif // SOLUTION_HPP_
