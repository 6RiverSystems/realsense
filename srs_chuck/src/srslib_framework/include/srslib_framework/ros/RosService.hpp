/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef ROSSERVICE_HPP_
#define ROSSERVICE_HPP_

#include <string>
using namespace std;

#include <ros/ros.h>

namespace srs {

class RosService
{
public:
    RosService(ros::NodeHandle rosHandle, string serviceName = "") :
        newRequestPending_(false),
        rosNodeHandle_(rosHandle),
        serviceName_(serviceName)
    {}

    virtual ~RosService()
    {
        rosServiceServer_.shutdown();
    }

    bool connectService()
    {
        connected_ = connect();
        return connected_;
    }

    string getServiceName() const
    {
        return serviceName_;
    }

    bool isServiceConnected() const
    {
        return connected_;
    }

    virtual bool newRequestPending() const
    {
        return newRequestPending_;
    }

    virtual void reset()
    {}

protected:
    ros::NodeHandle rosNodeHandle_;
    ros::ServiceServer rosServiceServer_;

    virtual bool connect() = 0;

    void setNewRequest(bool newValue)
    {
        newRequestPending_ = newValue;
    }

private:
    bool connected_;

    bool newRequestPending_;

    string serviceName_;
};

} // namespace srs

#endif // ROSSERVICE_HPP_
