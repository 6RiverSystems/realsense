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

class BaseRos
{
public:
    BaseRos(string name, int argc, char** argv)
    {
        ros::init(argc, argv, name);
        ros::Time::init();

        ROS_INFO_STREAM("Base ROS " << name << " started");
    }

    ~BaseRos()
    {}
};

} // namespace srs
