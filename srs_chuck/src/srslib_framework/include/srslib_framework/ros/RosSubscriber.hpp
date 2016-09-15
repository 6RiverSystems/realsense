/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
using namespace std;

#include <ros/ros.h>

namespace srs {

template<typename MESSAGE>
class RosSubscriber
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

        receiveData(message);
    }

private:
    ros::Subscriber dataSubscriber_;

    bool hasNeverReported_;

    bool newData_;

    ros::NodeHandle rosNodeHandle_;

    string topic_;
};

} // namespace srs
