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

//#include <filter/Measurement.hpp>

namespace srs {

class RosSensor
{
public:
    RosSensor(string name) :
        name_(name)
    {}

    virtual ~RosSensor()
    {}

    string getName() const
    {
        return name_;
    }

    virtual bool newDataAvailable() const = 0;

    virtual void reset() = 0;

protected:
    ros::NodeHandle rosNodeHandle_;
    ros::Subscriber rosSubscriber_;

private:
    string name_;
};

} // namespace srs

#endif // ROSSENSOR_HPP_
