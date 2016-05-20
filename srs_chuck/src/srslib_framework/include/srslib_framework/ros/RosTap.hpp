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
    RosTap(ros::NodeHandle rosHandle, string tapName = "") :
        connected_(false),
        newData_(false),
        rosNodeHandle_(rosHandle),
        tapName_(tapName)
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

    string getTapName() const
    {
        return tapName_;
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

    void setNewData(bool newValue)
    {
        newData_ = newValue;
    }

private:
    bool connected_;

    bool newData_;

    string tapName_;
};

} // namespace srs

#endif // ROSTAP_HPP_
