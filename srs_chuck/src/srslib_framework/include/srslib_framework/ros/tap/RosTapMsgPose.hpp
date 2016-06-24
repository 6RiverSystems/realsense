/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef ROSTAPMSGPOSE_HPP_
#define ROSTAPMSGPOSE_HPP_

#include <string>
using namespace std;

#include <srslib_framework/math/TimeMath.hpp>
#include <srslib_framework/robotics/Pose.hpp>
#include <srslib_framework/ros/RosTap.hpp>

#include <srslib_framework/MsgPose.h>
using namespace srslib_framework;

namespace srs {

class RosTapMsgPose :
    public RosTap
{
public:
    typedef typename Pose<>::BaseType BaseType;

    RosTapMsgPose(string topic, string description = "Pose Tap") :
        RosTap(topic, description)
    {}

    ~RosTapMsgPose()
    {
        disconnectTap();
    }

    Pose<> getPose()
    {
        setNewData(false);
        return currentPose_;
    }

    void reset()
    {
        RosTap::reset();
        set(TimeMath::time2number(ros::Time::now()), 0.0, 0.0, 0.0);
    }

    void set(double arrivalTime, BaseType x, BaseType y, BaseType theta)
    {
        currentPose_ = Pose<>(arrivalTime, x, y, theta);
        setNewData(true);
    }

protected:
    bool connect()
    {
        rosSubscriber_ = rosNodeHandle_.subscribe(getTopic(), 10, &RosTapMsgPose::onPose, this);
        return true;
    }

private:
    void onPose(const MsgPoseConstPtr message)
    {
        set(TimeMath::time2number(message->header.stamp),
            message->x,
            message->y,
            AngleMath::deg2rad<double>(message->theta));
    }

    Pose<> currentPose_;
};

} // namespace srs

#endif // ROSTAPMSGPOSE_HPP_
