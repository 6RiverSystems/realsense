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

    TYPE find(TYPE item) const;

    friend ostream& operator<<(ostream& stream,
        const MappedPriorityQueue<TYPE, PRIORITY, HASH, EQUAL_TO>& queue)
    {
        stream << "{" << endl;

        if (queue.empty())
        {
            stream << "  (empty)";
        }
        else
        {
            queue.printBuckets(stream);
            stream << endl;
            queue.printIndex(stream);
        }

        cout << endl << "Empty buckets: " << queue.sizeEmptyBuckets() << endl;
        cout << "Occupied buckets: " << queue.sizeOccuppiedBuckets() << endl << "}";

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
        return emptyBucketsCounter_;
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
    typedef unordered_map<TYPE, PRIORITY, HASH, EQUAL_TO> IndexType;
    typedef map<PRIORITY, BucketType*, less<PRIORITY>> QueueType;

    inline BucketType* getAvailableBucket()
    {
        BucketType* bucket = nullptr;
        if (!emptyBuckets_.empty())
        {
            emptyBucketsCounter_--;

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

    inline void recycleBucket(BucketType* bucket)
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

    BucketPoolType emptyBuckets_;
    int emptyBucketsCounter_;

    IndexType index_;

    QueueType queue_;
};

} // namespace srs

#include <datastructure/queue/MappedPriorityQueue.cpp>