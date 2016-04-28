/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef ROSSENSOR_HPP_
#define ROSSENSOR_HPP_

#include <string>
using namespace std;

#include <ros/ros.h>

namespace srs {

class RosTap
{
public:
    RosTap(string name) :
        connected_(false),
        name_(name),
        newData_(false),
        timestamp_(0)
    {}

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

    string getName() const
    {
        return name_;
    }

    uint32_t getTimestamp() const
    {
        return timestamp_;
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
    {}

protected:
    ros::NodeHandle rosNodeHandle_;
    ros::Subscriber rosSubscriber_;

    virtual bool connect() = 0;

    virtual bool disconnect()
    {
        rosSubscriber_.shutdown();
        return true;
    }

    void setNewData(bool newValue, uint32_t timestamp)
    {
        newData_ = newValue;
        timestamp_ = timestamp;
    }

private:
    bool connected_;
    string name_;
    bool newData_;
    uint32_t timestamp_;
};

} // namespace srs

#endif // ROSSENSOR_HPP_