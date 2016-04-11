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

class RosSensor
{
public:
    RosSensor(string name) :
        name_(name)
    {}

    virtual ~RosSensor()
    {}

    virtual void reset() = 0;

    string getName() const
    {
        return name_;
    }

protected:
    ros::NodeHandle rosNodeHandle_;

private:
    string name_;
};

} // namespace srs

#endif // ROSSENSOR_HPP_
