/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef PRIORITYQUEUE_HPP_
#define PRIORITYQUEUE_HPP_

#include <queue>
using namespace std;

namespace srs {

template<typename TYPE, typename PRIORITY = int>
class PriorityQueue
{
    typedef pair<PRIORITY, TYPE> ElementType;

    inline bool empty() const
    {
        return elements_.empty();
    }

    inline TYPE pop()
    {
        TYPE topPriorityElement = elements_.top().second;

        elements_.pop();
        return topPriorityElement;
    }

    inline void push_back(TYPE item, PRIORITY priority)
    {
        elements_.emplace(priority, item);
    }

private:
    priority_queue<ElementType, vector<ElementType>, std::greater<ElementType>> elements_;
};

} // namespace srs

#endif // PRIORITYQUEUE_HPP_
