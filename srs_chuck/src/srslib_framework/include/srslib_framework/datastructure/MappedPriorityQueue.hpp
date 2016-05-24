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
    // TODO: Recycle the empty buckets to improve performance and memory management

    bool empty() const
    {
        return indexMap_.empty();
    }

    void erase(TYPE item)
    {
        PRIORITY priority = indexMap_[item];
        indexMap_.erase(item);

        BucketType* bucket = priorityQueue_[priority];
        bucket->erase(item);

        if (bucket->empty())
        {
            priorityQueue_.erase(priority);
            delete bucket;
        }
    }

    bool exists(TYPE item)
    {
        auto result = indexMap_.find(item);
        return result != indexMap_.end();
    }

    TYPE find(TYPE item)
    {
        auto result = indexMap_.find(item);
        if (result == indexMap_.end())
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
//        stream << "Buckets" << endl;
//        for (auto bucketIterator : queue.priorityQueue_)
//        {
//            stream << bucketIterator.first << ": {" << endl;
//            for (auto item : *bucketIterator.second)
//            {
//                stream << item << '\n';
//            }
//            stream << "}" << '\n';
//        }
//
//        stream << "Index map" << endl;
//        for (auto item : queue.indexMap_)
//        {
//            stream << "(" << item.first << "  priority: " << item.second << ")\n";
//        }
//
//        stream << "}";

        return stream;
    }

    bool pop(TYPE& item)
    {
        if (indexMap_.empty())
        {
            return false;
        }

        auto lowestBucketIterator = priorityQueue_.begin();
        BucketType* bucket = lowestBucketIterator->second;
        PRIORITY key = lowestBucketIterator->first;

        auto itemIterator = bucket->begin();

        item = *itemIterator;
        bucket->erase(itemIterator);

        if (bucket->empty())
        {
            priorityQueue_.erase(key);
            delete bucket;
        }

        indexMap_.erase(item);

        return true;
    }

    void push(PRIORITY priority, TYPE item)
    {
        auto bucketIterator = priorityQueue_.find(priority);

        BucketType* bucket;

        if (bucketIterator != priorityQueue_.end())
        {
            bucket = bucketIterator->second;
        }
        else
        {
            bucket = new BucketType();
        }

        bucket->insert(item);

        priorityQueue_[priority] = bucket;
        indexMap_[item] = priority;
    }

private:
    typedef unordered_set<TYPE> BucketType;
    typedef map<PRIORITY, BucketType*, less<PRIORITY>> MapType;

    MapType priorityQueue_;
    unordered_map<TYPE, PRIORITY, HASH> indexMap_;
};

} // namespace srs

#endif // MAPPEDPRIORITYQUEUE_HPP_
