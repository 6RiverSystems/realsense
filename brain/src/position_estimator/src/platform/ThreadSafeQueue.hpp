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
        std::lock_guard<std::mutex> lock(other.mutex_);
        dataQueue_ = other.dataQueue_;
    }

    bool empty() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return dataQueue_.empty();
    }

    std::shared_ptr<T> pop()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (dataQueue_.empty())
        {
            return std::shared_ptr<T>();
        }

        std::shared_ptr<T> result(std::make_shared<T>(dataQueue_.front()));
        dataQueue_.pop();

        return result;
    }

    void push(T newValue)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        dataQueue_.push(newValue);
        dataCondition_.notify_one();
    }

    unsigned int size() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return dataQueue_.size();
    }

    std::shared_ptr<T> blockingPop()
    {
        std::unique_lock<std::mutex> lock(mutex_);

        dataCondition_.wait(lock, [this]{return !dataQueue_.empty();});

        std::shared_ptr<T> result(std::make_shared<T>(dataQueue_.front()));
        dataQueue_.pop();

        return result;
    }

private:
    mutable std::mutex mutex_;
    std::queue<T> dataQueue_;
    std::condition_variable dataCondition_;
};

} // namespace srs

#endif // THREADSAFEQUEUE_HPP_
