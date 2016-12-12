/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/ros/unit/BaseRos.hpp>

namespace srs {

template<class T>
class RosUnit : public BaseRos
{
public:
    RosUnit(string name, int argc, char** argv, double refreshRate);
    virtual ~RosUnit();

    void run();

protected:
    virtual void execute()
    {}

    virtual void initialize()
    {}

    void getParamFromServer(string service, string parameter, float& value, float defaultValue);
    void getParamFromEnv(string parameter, string& value, string defaultValue);

private:
    ros::Time currentTime_;

    ros::Time previousTime_;

    ros::Rate rate_;
    double refreshRateFrequency_;
    ros::NodeHandle rosNodeHandle_;
};

} // namespace srs

#include <ros/unit/RosUnit.cpp>
