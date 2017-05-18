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

template <typename NODETYPE>
class Plan : public list<NODETYPE*>
{
public:
    Plan() :
        list<NODETYPE*>(),
        closedNodesCount_(0),
        openNodesCount_(0),
        totalCost_(0),
        valid_(false)
    {}

    void clear()
    {
        list<NODETYPE*>::clear();

        closedNodesCount_ = 0;
        openNodesCount_ = 0;
        valid_ = false;
    }

    unsigned int getClosedNodesCount() const
    {
        return closedNodesCount_;
    }

    NODETYPE* getGoal() const
    {
        return list<NODETYPE*>::back();
    }

    unsigned int getOpenNodesCount() const
    {
        return openNodesCount_;
    }

    NODETYPE* getStart() const
    {
        return list<NODETYPE*>::front();
    }

    int getTotalCost() const
    {
        return totalCost_;
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

    void setTotalCost(int newValue)
    {
        totalCost_ = newValue;
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

    int totalCost_;

    bool valid_;
};

} // namespace srs
