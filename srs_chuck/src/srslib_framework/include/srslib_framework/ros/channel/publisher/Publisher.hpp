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

template<typename MESSAGE, typename TYPE>
class Publisher
{
public:
    Publisher(string topic,
            unsigned int queueLength,
            bool latched,
            string nameSpace) :
        topic_(topic)
    {
        rosNodeHandle_ = ros::NodeHandle(nameSpace);
        dataPublisher_ = rosNodeHandle_.advertise<MESSAGE>(topic_, queueLength, latched);
    }

    virtual ~Publisher()
    {}

    string getTopic() const
    {
        return topic_;
    }

    virtual void publish(TYPE data)
    {
        publishMessage(convertData(data));
    }

    virtual MESSAGE convertData(TYPE data) = 0;

protected:
    void publishMessage(const MESSAGE& message)
    {
        dataPublisher_.publish(message);
    }

private:
    ros::Publisher dataPublisher_;

    ros::NodeHandle rosNodeHandle_;

    string topic_;
};

} // namespace srs
