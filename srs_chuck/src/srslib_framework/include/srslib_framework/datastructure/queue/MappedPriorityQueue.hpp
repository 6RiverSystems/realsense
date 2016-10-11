/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <map>
#include <unordered_map>
#include <unordered_set>
#include <forward_list>
using namespace std;

namespace srs {

template<typename TYPE,
    typename PRIORITY = int,
    typename HASH = std::hash<TYPE>,
    typename EQUAL_TO = std::equal_to<TYPE>,
    int BUCKETS_INITIAL = 100,
    int BUCKETS_MAX = 10000>
class MappedPriorityQueue
{
public:
    MappedPriorityQueue();
    ~MappedPriorityQueue();

    void clear();

    bool empty() const
    {
        return index_.empty();
    }

    void erase(TYPE item);

    bool exists(TYPE item) const
    {
        auto result = index_.find(item);
        return result != index_.end();
    }

    TYPE find(TYPE& item) const;

    friend ostream& operator<<(ostream& stream,
        const MappedPriorityQueue<TYPE, PRIORITY, HASH, EQUAL_TO>& queue)
    {
        stream << "MappedPriorityQueue {" << endl;

        if (queue.empty())
        {
            stream << "  (empty)";
        }
        else
        {
            queue.printBuckets(stream);
            queue.printIndex(stream);
        }

        cout << endl << "Empty buckets: " << queue.sizeEmptyBuckets() << endl;
        cout << "Occupied buckets: " << queue.sizeOccuppiedBuckets() << endl;

        return stream;
    }

    bool pop(TYPE& item);
    void push(PRIORITY priority, TYPE item);

    size_t size() const
    {
        return index_.size();
    }

    size_t sizeEmptyBuckets() const
    {
        return distance(emptyBuckets_.begin(), emptyBuckets_.end());
    }

    size_t sizeInitialBuckets() const
    {
        return BUCKETS_INITIAL;
    }

    size_t sizeMaxBuckets() const
    {
        return BUCKETS_MAX;
    }

    size_t sizeOccuppiedBuckets() const
    {
        return queue_.size();
    }

private:
    typedef unordered_set<TYPE, HASH, EQUAL_TO> BucketType;
    typedef forward_list<BucketType*> BucketPoolType;

    BucketType* getNewBucket()
    {
        BucketType* bucket = nullptr;
        if (!emptyBuckets_.empty())
        {
            bucket = emptyBuckets_.front();
            emptyBuckets_.pop_front();
        }
        else
        {
            bucket = new BucketType();
        }

        return bucket;
    }

    void printBuckets(ostream& stream) const;
    void printIndex(ostream& stream) const;

    void recycleBucket(BucketType* bucket)
    {
        // Recycle the bucket only if the number of empty
        // buckets in the queue is less than the
        // specified threshold
        if (emptyBucketsCounter_ < BUCKETS_MAX)
        {
            emptyBucketsCounter_++;
            emptyBuckets_.push_front(bucket);
        }
        else
        {
            delete bucket;
        }
    }

    map<PRIORITY, BucketType*, less<PRIORITY>> queue_;
    unordered_map<TYPE, PRIORITY, HASH, EQUAL_TO> index_;

    BucketPoolType emptyBuckets_;
    int emptyBucketsCounter_;
};

} // namespace srs

#include <datastructure/queue/MappedPriorityQueue.cpp>
