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
    static Solution<SOLUTION_ITEM>* instanceOfValidEmpty()
    {
        return new Solution(true);
    }

    static Solution<SOLUTION_ITEM>* instanceOfInvalidEmpty()
    {
        return new Solution(false);
    }

    static Solution<SOLUTION_ITEM>* instanceOfValid(SOLUTION_ITEM firstNode)
    {
        Solution<SOLUTION_ITEM>* solution = new Solution(true);
        solution->push_back(firstNode);

        return solution;
    }

    void append(Solution<SOLUTION_ITEM>* other)
    {
        list<SOLUTION_ITEM>::insert(list<SOLUTION_ITEM>::end(), other->begin(), other->end());
    }

    unsigned int getExploredNodes() const
    {
        return exploredNodes_;
    }

    SOLUTION_ITEM getGoal() const
    {
        return list<SOLUTION_ITEM>::back();
    }

    SOLUTION_ITEM getStart() const
    {
        return list<SOLUTION_ITEM>::front();
    }

    bool isValid() const
    {
        return valid_;
    }

    friend ostream& operator<<(ostream& stream, const Solution& solution)
    {
        return stream << solution.toString();
    }

    void setExploredNodes(unsigned int newValue)
    {
        exploredNodes_ = newValue;
    }

    void setValid(bool newValue)
    {
        valid_ = newValue;
    }

    string toString() const
    {
        stringstream stream;

        int counter = 0;

        stream << "{" << endl;
        stream << "v: " << valid_ << ", en: " << exploredNodes_ << endl;
        for (auto node : *this)
        {
            stream << setw(4) << counter++ << ": " << node << endl;
        }

        stream << "}";

        return stream.str();
    }

protected:
    Solution(bool valid) :
        exploredNodes_(0),
        valid_(valid)
    {}

private:
    unsigned int exploredNodes_;

    bool valid_;
};

} // namespace srs
