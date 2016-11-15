/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
using namespace std;

#include <ros/ros.h>

#include <srslib_framework/ros/tap/subscriber/Subject.hpp>

namespace srs {

template<typename MESSAGE>
class RosSubscriber : public Subject<RosSubscriber<MESSAGE>>
{
public:
    RosSubscriber(string topic,
        unsigned int queueLength,
        string nameSpace) :
        topic_(topic),
        hasNeverReported_(true),
        newData_(false)
    {
        rosNodeHandle_ = ros::NodeHandle(nameSpace);
        dataSubscriber_ = rosNodeHandle_.subscribe(topic_, queueLength,
            &RosSubscriber::onDataReceived, this);
    }

    virtual ~RosSubscriber()
    {
        dataSubscriber_.shutdown();
    }

    string getTopic() const
    {
        return topic_;
    }

    bool hasNeverReported() const
    {
        return hasNeverReported_;
    }

    virtual bool newDataAvailable() const
    {
        return newData_;
    }

    virtual void receiveData(const typename MESSAGE::ConstPtr message) = 0;

    virtual void reset()
    {
        hasNeverReported_ = true;
        declareStale();
    }

    virtual void set()
    {
        hasNeverReported_ = false;
        declareNew();
    }

protected:
    void declareNew()
    {
        newData_ = true;
    }

    void declareStale()
    {
        newData_ = false;
    }

    void onDataReceived(const typename MESSAGE::ConstPtr message)
    {
        hasNeverReported_ = false;

        // Let the specific subscriber to decode the message
        receiveData(message);

        // Notify all the observers that some
        // some new data is available
        Subject<RosSubscriber<MESSAGE>>::notify();
    }

private:
    ros::Subscriber dataSubscriber_;

    bool hasNeverReported_;

    bool newData_;

    ros::NodeHandle rosNodeHandle_;

    string topic_;
};

} // namespace srs
