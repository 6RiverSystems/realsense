namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename TYPE, typename PRIORITY,
    typename HASH, typename EQUAL_TO,
    int BUCKETS_INITIAL, int BUCKETS_MAX>
MappedPriorityQueue<TYPE, PRIORITY, HASH, EQUAL_TO, BUCKETS_INITIAL, BUCKETS_MAX>::
    MappedPriorityQueue()
{
    // Allocate an initial number of empty buckets
    for (int b = BUCKETS_INITIAL; b > 0; --b)
    {
        emptyBuckets_.push_front(new BucketType());
    }

    emptyBucketsCounter_ = BUCKETS_INITIAL;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename TYPE, typename PRIORITY,
    typename HASH, typename EQUAL_TO,
    int BUCKETS_INITIAL, int BUCKETS_MAX>
MappedPriorityQueue<TYPE, PRIORITY, HASH, EQUAL_TO, BUCKETS_INITIAL, BUCKETS_MAX>::
    ~MappedPriorityQueue()
{
    // Remove all the empty buckets
    for (auto bucket : emptyBuckets_)
    {
        delete bucket;
    }
    emptyBuckets_.clear();

    // Destroy all the non-empty buckets
    for (auto bucket : queue_)
    {
        delete bucket.second;
    }
    queue_.clear();

    index_.clear();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename TYPE, typename PRIORITY,
    typename HASH, typename EQUAL_TO,
    int BUCKETS_INITIAL, int BUCKETS_MAX>
void MappedPriorityQueue<TYPE, PRIORITY, HASH, EQUAL_TO, BUCKETS_INITIAL, BUCKETS_MAX>::
    clear()
{
    // Clear and recycle each occupied bucket
    for (auto bucket : queue_)
    {
        bucket.second->clear();
        recycleBucket(bucket.second);
    }

    // Clear all the references to the occupied buckets
    queue_.clear();
    index_.clear();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename TYPE, typename PRIORITY,
    typename HASH, typename EQUAL_TO,
    int BUCKETS_INITIAL, int BUCKETS_MAX>
void MappedPriorityQueue<TYPE, PRIORITY, HASH, EQUAL_TO, BUCKETS_INITIAL, BUCKETS_MAX>::
    erase(TYPE item)
{
    auto itemIterator = index_.find(item);
    if (itemIterator == index_.end())
    {
        return;
    }

    // Find the bucket which contains the item
    auto bucketIterator = queue_.find(itemIterator->second);

    // Remove the item from the bucket
    bucketIterator->second->erase(itemIterator->first);

    // Recycle the bucket if it becomes empty
    if (bucketIterator->second->empty())
    {
        queue_.erase(bucketIterator);
        recycleBucket(bucketIterator->second);
    }

    // Remove the item from the general index
    index_.erase(itemIterator->first);

}

////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename TYPE, typename PRIORITY,
    typename HASH, typename EQUAL_TO,
    int BUCKETS_INITIAL, int BUCKETS_MAX>
TYPE MappedPriorityQueue<TYPE, PRIORITY, HASH, EQUAL_TO, BUCKETS_INITIAL, BUCKETS_MAX>::
    find(TYPE item) const
{
    auto result = index_.find(item);
    if (result == index_.end())
    {
        return TYPE();
    }

    return result->first;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename TYPE, typename PRIORITY,
    typename HASH, typename EQUAL_TO,
    int BUCKETS_INITIAL, int BUCKETS_MAX>
bool MappedPriorityQueue<TYPE, PRIORITY, HASH, EQUAL_TO, BUCKETS_INITIAL, BUCKETS_MAX>::
    pop(TYPE& item)
{
    if (index_.empty())
    {
        return false;
    }

    // Find the bucket that contains the lowest priority
    auto lowestBucketIterator = queue_.begin();
    BucketType* bucket = lowestBucketIterator->second;
    PRIORITY key = lowestBucketIterator->first;

    // Find the first element of the bucket and return it
    auto firstItemIterator = bucket->begin();
    item = *firstItemIterator;

    // Remove the item from the bucket
    // and recycle it if empty
    bucket->erase(firstItemIterator);
    if (bucket->empty())
    {
        queue_.erase(key);
        recycleBucket(bucket);
    }

    // Remove the item from the map index
    index_.erase(item);

    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename TYPE, typename PRIORITY,
    typename HASH, typename EQUAL_TO,
    int BUCKETS_INITIAL, int BUCKETS_MAX>
void MappedPriorityQueue<TYPE, PRIORITY, HASH, EQUAL_TO, BUCKETS_INITIAL, BUCKETS_MAX>::
    push(PRIORITY priority, TYPE item)
{
    // Find the bucket that will contain the item
    auto bucketIterator = queue_.find(priority);

    BucketType* bucket;
    if (bucketIterator != queue_.end())
    {
        bucket = bucketIterator->second;
    }
    else
    {
        // If no bucket was found, get one
        // from the original pool and link it
        // to the queue of priorities
        bucket = getAvailableBucket();
        queue_[priority] = bucket;
    }

    // Add the item to the bucket and to the map index
    bucket->insert(item);
    index_[item] = priority;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename TYPE, typename PRIORITY,
    typename HASH, typename EQUAL_TO,
    int BUCKETS_INITIAL, int BUCKETS_MAX>
void MappedPriorityQueue<TYPE, PRIORITY, HASH, EQUAL_TO, BUCKETS_INITIAL, BUCKETS_MAX>::
    printBuckets(ostream& stream) const
{
    int counter = 0;

    stream << "Buckets" << endl;
    for (auto bucketIterator : queue_)
    {
        stream << bucketIterator.first << ": {" << endl;
        for (auto item : *bucketIterator.second)
        {
            stream << setw(4) << counter++ << ": " << item << endl;
        }
        stream << "}" << endl;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename TYPE, typename PRIORITY,
    typename HASH, typename EQUAL_TO,
    int BUCKETS_INITIAL, int BUCKETS_MAX>
void MappedPriorityQueue<TYPE, PRIORITY, HASH, EQUAL_TO, BUCKETS_INITIAL, BUCKETS_MAX>::
    printIndex(ostream& stream) const
{
    int counter = 0;

    stream << "Index map" << endl;
    for (auto itemIterator : index_)
    {
        stream << setw(4) << counter++ << ": [" <<
            setw(10) << itemIterator.second << "] " <<
            itemIterator.first << endl;
    }
}

} // namespace srs
