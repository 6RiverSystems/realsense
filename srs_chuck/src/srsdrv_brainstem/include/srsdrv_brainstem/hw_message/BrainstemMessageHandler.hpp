/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef BRAINSTEMMESSAGEHANDLER_HPP_
#define BRAINSTEMMESSAGEHANDLER_HPP_

#include <string>
using namespace std;

#include <ros/ros.h>

namespace srs {

class BrainstemMessageHandler
{
public:
    BrainstemMessageHandler(char messageKey, string nameSpace = "~") :
        messageKey_(messageKey)
    {
        rosNodeHandle_ = ros::NodeHandle(nameSpace);
    }

    virtual ~BrainstemMessageHandler()
    {}

    virtual bool isKeyMatching(char key) const
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

#endif // BRAINSTEMMESSAGEHANDLER_HPP_
