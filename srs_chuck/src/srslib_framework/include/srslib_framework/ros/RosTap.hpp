/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef ROSTAP_HPP_
#define ROSTAP_HPP_

#include <string>
using namespace std;

#include <ros/ros.h>

namespace srs {

class RosTap
{
public:
    RosTap(string topic, string description = "") :
        connected_(false),
        hasNeverReported_(true),
        newData_(false),
        description_(description),
        topic_(topic)
    {
        rosNodeHandle_ = ros::NodeHandle();
    }

    virtual ~RosTap()
    {}

    bool connectTap()
    {
        connected_ = connect();
        return connected_;
    }

    bool disconnectTap()
    {
        connected_ = disconnect();
        return connected_;
    }

    string getDescription() const
    {
        return description_;
    }

    string getTopic() const
    {
        return topic_;
    }

    bool hasNeverReported() const
    {
        return hasNeverReported_;
    }

    bool isTapConnected() const
    {
        return connected_;
    }

    virtual bool newDataAvailable() const
    {
        return newData_;
    }

    virtual void reset()
    {
    	setNewData(false);
    }

protected:
    ros::NodeHandle rosNodeHandle_;
    ros::Subscriber rosSubscriber_;

    virtual bool connect() = 0;

    virtual bool disconnect()
    {
        rosSubscriber_.shutdown();
        return true;
    }

    void setNewData(bool newValue)
    {
        newData_ = newValue;
        hasNeverReported_ = false;
    }

private:
    bool connected_;

    string description_;

    bool hasNeverReported_;

    bool newData_;

    string topic_;
};

} // namespace srs

#endif // ROSTAP_HPP_
