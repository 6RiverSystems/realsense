/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
using namespace std;

#include <ros/ros.h>

#include <srslib_framework/ros/RosSubscriber.hpp>

namespace srs {

template<typename MESSAGE, typename TYPE>
class RosSubscriberSingleData : public RosSubscriber<MESSAGE>
{
public:
    RosSubscriberSingleData(string topic,
        unsigned int queueLength,
        string nameSpace) :
            RosSubscriber<MESSAGE>(topic, queueLength, nameSpace),
            data_(TYPE())
    {}

    virtual ~RosSubscriberSingleData()
    {}

    TYPE peek() const
    {
        return data_;
    }

    TYPE pop()
    {
        RosSubscriber<MESSAGE>::declareStale();
        return data_;
    }

    virtual void reset()
    {
        RosSubscriber<MESSAGE>::reset();

        data_ = TYPE();
    }

    virtual void set(TYPE data)
    {
        RosSubscriber<MESSAGE>::set();

        data_ = data;
    }

private:
    TYPE data_;
};

} // namespace srs
