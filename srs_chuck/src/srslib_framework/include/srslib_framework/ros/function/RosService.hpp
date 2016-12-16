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
    RosService(string name, string description = "", string nameSpace = "~") :
        connected_(false),
        description_(description),
        name_(name),
        newRequestPending_(false)
    {
        rosNodeHandle_ = ros::NodeHandle(nameSpace);
    }

    virtual ~RosService()
    {
        rosServiceServer_.shutdown();
    }

    bool connectService()
    {
        connected_ = connect();
        return connected_;
    }

    string getDescription() const
    {
        return description_;
    }

    string getName() const
    {
        return name_;
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

    string description_;

    bool newRequestPending_;

    string name_;
};

} // namespace srs

#endif // ROSSERVICE_HPP_
