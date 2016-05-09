/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef MAPPEDPRIORITYQUEUE_HPP_
#define MAPPEDPRIORITYQUEUE_HPP_

#include <map>
#include <unordered_map>
#include <unordered_set>
using namespace std;

namespace srs {

template<typename TYPE,
    typename PRIORITY = int,
    typename HASH = std::hash<TYPE>,
    typename EQUAL_TO = std::equal_to<TYPE>>
class MappedPriorityQueue
{
public:
    // TODO: Implement begin() and end() for foreach operations

    bool empty() const
    {
        return itemsMap_.empty();
    }

    void erase(TYPE item)
    {
        PRIORITY priority = itemsMap_[item];
        itemsMap_.erase(item);

        BucketType& bucket = priorityQueue_[priority];
        bucket.erase(item);
    }

    bool exists(TYPE item)
    {
        return itemsMap_.count(item);
    }

    TYPE find(TYPE item)
    {
        auto result = itemsMap_.find(item);
        if (result == itemsMap_.end())
        {
            return TYPE();
        }

        return result->first;
    }

    friend ostream& operator<<(ostream& stream, const MappedPriorityQueue& queue)
    {
        for (auto bucket : queue.priorityQueue_)
        {
            for (auto item : bucket.second)
            {
                stream << bucket.first << ": " << item << '\n';
            }
        }

//        stream << "MappedPriorityQueue {" << '\n';
//        for (auto bucket : queue.priorityQueue_)
//        {
//            stream << bucket.first << ": {" << endl;
//            for (auto item : bucket.second)
//            {
//                stream << item << '\n';
//            }
//            stream << "}" << '\n';
//        }
//
//        stream << "Items map" << endl;
//        for (auto item : queue.itemsMap_)
//        {
//            stream << "(\n" <<
//                item.first << '\n' <<
//                "priority: " << item.second << "\n" <<
//                ")\n";
//        }
//
//        stream << "}";

        return stream;
    }

    bool pop(TYPE& item)
    {
        if (itemsMap_.empty())
        {
            return false;
        }

        auto mapIter = priorityQueue_.begin();
        BucketType& bucket = mapIter->second;

        auto bucketIter = bucket.begin();
        item = *bucketIter;

        bucket.erase(bucketIter);
        itemsMap_.erase(item);
        if (bucket.empty())
        {
            priorityQueue_.erase(mapIter);
        }

        return true;
    }

    void push(PRIORITY priority, TYPE item)
    {
        BucketType& bucket = priorityQueue_[priority];
        bucket.insert(item);

        itemsMap_[item] = priority;
    }

private:
    typedef unordered_set<TYPE> BucketType;
    typedef map<PRIORITY, BucketType, less<PRIORITY>> MapType;

    MapType priorityQueue_;
    unordered_map<TYPE, PRIORITY, HASH> itemsMap_;
};

} // namespace srs

#endif // MAPPEDPRIORITYQUEUE_HPP_
