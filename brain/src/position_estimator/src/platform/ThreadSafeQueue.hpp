/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef THREADSAFEQUEUE_HPP_
#define THREADSAFEQUEUE_HPP_

#include <queue>
#include <memory>
#include <mutex>
#include <condition_variable>
using namespace std;

namespace srs {

template<typename T>
class ThreadSafeQueue
{
public:
    ThreadSafeQueue()
    {
    }

    ThreadSafeQueue(ThreadSafeQueue const& other)
    {
        lock_guard<mutex> lock(other.mutex_);
        dataQueue_ = other.dataQueue_;
    }

    bool empty() const
    {
        lock_guard<mutex> lock(mutex_);
        return dataQueue_.empty();
    }

    shared_ptr<T> pop()
    {
        lock_guard<mutex> lock(mutex_);
        if (dataQueue_.empty())
        {
            return shared_ptr<T>();
        }

        shared_ptr<T> result(make_shared<T>(dataQueue_.front()));
        dataQueue_.pop();

        return result;
    }

    void push(T newValue)
    {
        lock_guard<mutex> lock(mutex_);
        dataQueue_.push(newValue);
        dataCondition_.notify_one();
    }

    unsigned int size() const
    {
        lock_guard<mutex> lock(mutex_);
        return dataQueue_.size();
    }

    shared_ptr<T> blockingPop()
    {
        unique_lock<mutex> lock(mutex_);

        dataCondition_.wait(lock, [this]{return !dataQueue_.empty();});

        shared_ptr<T> result(make_shared<T>(dataQueue_.front()));
        dataQueue_.pop();

        return result;
    }

private:
    mutable mutex mutex_;
    queue<T> dataQueue_;
    condition_variable dataCondition_;
};

} // namespace srs

#endif // THREADSAFEQUEUE_HPP_
