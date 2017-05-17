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

    nodeIndex_.reserve(BUCKETS_MAX);
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
    for (auto bucket : nodeQueue_)
    {
        delete bucket.second;
    }
    nodeQueue_.clear();

    nodeIndex_.clear();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename TYPE, typename PRIORITY,
    typename HASH, typename EQUAL_TO,
    int BUCKETS_INITIAL, int BUCKETS_MAX>
void MappedPriorityQueue<TYPE, PRIORITY, HASH, EQUAL_TO, BUCKETS_INITIAL, BUCKETS_MAX>::
    clear()
{
    // Clear and recycle each occupied bucket
    for (auto bucket : nodeQueue_)
    {
        bucket.second->clear();
        recycleBucket(bucket.second);
    }

    // Clear all the references to the occupied buckets
    nodeQueue_.clear();
    nodeIndex_.clear();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename TYPE, typename PRIORITY,
    typename HASH, typename EQUAL_TO,
    int BUCKETS_INITIAL, int BUCKETS_MAX>
void MappedPriorityQueue<TYPE, PRIORITY, HASH, EQUAL_TO, BUCKETS_INITIAL, BUCKETS_MAX>::
    erase(TYPE item)
{
    auto itemIterator = nodeIndex_.find(item);
    if (itemIterator == nodeIndex_.end())
    {
        return;
    }

    BucketType* bucket = itemIterator->second->bucket;
    auto it = itemIterator->second;
    bucket->erase(it);

    if (bucket->empty())
    {
        PRIORITY key = itemIterator->second->priority;
        nodeQueue_.erase(key);
        recycleBucket(bucket);
    }

    nodeIndex_.erase(itemIterator);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename TYPE, typename PRIORITY,
    typename HASH, typename EQUAL_TO,
    int BUCKETS_INITIAL, int BUCKETS_MAX>
TYPE MappedPriorityQueue<TYPE, PRIORITY, HASH, EQUAL_TO, BUCKETS_INITIAL, BUCKETS_MAX>::
    find(TYPE item) const
{
    auto result = nodeIndex_.find(item);
    if (result == nodeIndex_.end())
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
    if (nodeIndex_.empty())
    {
        return false;
    }

    // Find the bucket that contains the lowest priority
    auto lowestBucketIterator = nodeQueue_.begin();
    BucketType* bucket = lowestBucketIterator->second;

    // Find the first element of the bucket and return it
    item = bucket->begin()->info;

    // Remove the item from the bucket
    // and recycle it if empty
    bucket->pop_front();
    if (bucket->empty())
    {
        PRIORITY key = lowestBucketIterator->first;
        nodeQueue_.erase(key);
        recycleBucket(bucket);
    }

    // Remove the item from the map index
    nodeIndex_.erase(item);

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
    auto bucketIterator = nodeQueue_.find(priority);

    BucketType* bucket;
    if (bucketIterator != nodeQueue_.end())
    {
        bucket = bucketIterator->second;
    }
    else
    {
        // If no bucket was found, get one
        // from the original pool and link it
        // to the queue of priorities
        bucket = getAvailableBucket();
        nodeQueue_[priority] = bucket;
    }

    // Add the item to the bucket and to the map index
    auto it = bucket->emplace( bucket->end(), priority, item, bucket );
    nodeIndex_[item] = it;
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

    stream << "Buckets " << "{" << endl;
    for (auto bucketIterator : nodeQueue_)
    {
        stream << bucketIterator.first << ": {" << endl;
        for (auto item : *bucketIterator.second)
        {
            stream << setw(4) << counter++ << ": " << item << endl;
        }
        stream << "}" << endl;
    }
    stream << "}";
}

////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename TYPE, typename PRIORITY,
    typename HASH, typename EQUAL_TO,
    int BUCKETS_INITIAL, int BUCKETS_MAX>
void MappedPriorityQueue<TYPE, PRIORITY, HASH, EQUAL_TO, BUCKETS_INITIAL, BUCKETS_MAX>::
    printIndex(ostream& stream) const
{
    int counter = 0;

    stream << "Index map {" << endl;
    for (auto itemIterator : nodeIndex_)
    {
        stream << setw(4) << counter++ << ": [" <<
            setw(10) << itemIterator.second->priority << "] " <<
            itemIterator.first << endl;
    }
    stream << "}";
}

} // namespace srs
