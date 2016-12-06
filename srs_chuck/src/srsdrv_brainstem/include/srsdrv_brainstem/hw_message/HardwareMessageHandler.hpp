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

class HardwareMessageHandler
{
public:
    HardwareMessageHandler(char messageKey, string nameSpace = "~") :
        messageKey_(messageKey)
    {
        rosNodeHandle_ = ros::NodeHandle(nameSpace);
    }

    virtual ~HardwareMessageHandler()
    {}

    char getKey() const
    {
        return messageKey_;
    }

    bool isKeyMatching(char key) const
    {
        return key == messageKey_;
    }

    virtual void receiveData(ros::Time currentTime, vector<char>& binaryData) = 0;

protected:
    ros::NodeHandle rosNodeHandle_;

private:
    char messageKey_;
};

} // namespace srs
