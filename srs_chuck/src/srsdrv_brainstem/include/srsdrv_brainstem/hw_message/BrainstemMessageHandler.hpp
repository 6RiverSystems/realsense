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

// Define the macros for the beginning and end of a
// brainstem message. The macro automatically define the
// strict packing pragma and the struct for the
// message data
#define BRAINSTEM_MESSAGE_BEGIN(name) \
    _Pragma("pack(push, 1)") \
    struct name \
    {

#define BRAINSTEM_MESSAGE_END \
    }; \
    _Pragma("pack(pop)")

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

#endif // BRAINSTEMMESSAGEHANDLER_HPP_
