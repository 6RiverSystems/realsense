/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
using namespace std;

#include <ros/ros.h>

#include <srslib_framework/ros/tap/subscriber/Subscriber.hpp>

namespace srs {

template<typename MESSAGE, typename TYPE>
class SubscriberSingleData : public Subscriber<MESSAGE>
{
public:
    SubscriberSingleData(string topic,
        unsigned int queueLength,
        string nameSpace) :
            Subscriber<MESSAGE>(topic, queueLength, nameSpace),
            data_(TYPE())
    {}

    virtual ~SubscriberSingleData()
    {}

    TYPE peek() const
    {
        return data_;
    }

    TYPE pop()
    {
        Subscriber<MESSAGE>::declareStale();
        return data_;
    }

    virtual void reset()
    {
        Subscriber<MESSAGE>::reset();

        data_ = TYPE();
    }

    using Subscriber<MESSAGE>::set;

    virtual void set(TYPE data)
    {
        Subscriber<MESSAGE>::set();

        data_ = data;
    }

private:
    TYPE data_;
};

} // namespace srs
