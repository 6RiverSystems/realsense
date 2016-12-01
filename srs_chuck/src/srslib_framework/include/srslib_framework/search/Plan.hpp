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

#include <srslib_framework/search/SearchNode.hpp>

namespace srs {

class Plan : public list<SearchNode*>
{
public:
    Plan() :
        list<SearchNode*>(),
        closedNodesCount_(0),
        openNodesCount_(0),
        valid_(false)
    {}

    void clear()
    {
        list<SearchNode*>::clear();

        closedNodesCount_ = 0;
        openNodesCount_ = 0;
        valid_ = false;
    }

    unsigned int getClosedNodesCount() const
    {
        return closedNodesCount_;
    }

    SearchNode* getGoal() const
    {
        return list<SearchNode*>::back();
    }

    unsigned int getOpenNodesCount() const
    {
        return openNodesCount_;
    }

    SearchNode* getStart() const
    {
        return list<SearchNode*>::front();
    }

    bool isValid() const
    {
        return valid_;
    }

    friend ostream& operator<<(ostream& stream, const Plan& plan)
    {
        return stream << plan.toString();
    }

    void setClosedNodesCount(unsigned int newValue)
    {
        closedNodesCount_ = newValue;
    }

    void setOpenNodesCount(unsigned int newValue)
    {
        openNodesCount_ = newValue;
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
        stream << "v: " << valid_;
        stream << ", cn: " << closedNodesCount_;
        stream << ", on: " << openNodesCount_ << endl;
        for (auto node : *this)
        {
            stream << setw(4) << counter++ << ": " << node << endl;
        }

        stream << "}";

        return stream.str();
    }

private:
    unsigned int closedNodesCount_;

    unsigned int openNodesCount_;

    bool valid_;
};

} // namespace srs
