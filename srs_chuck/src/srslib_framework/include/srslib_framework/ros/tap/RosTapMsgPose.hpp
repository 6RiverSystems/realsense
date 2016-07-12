/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef ROSTAPMSGPOSE_HPP_
#define ROSTAPMSGPOSE_HPP_

#include <string>
using namespace std;

#include <srslib_framework/MsgPose.h>

#include <srslib_framework/math/TimeMath.hpp>

#include <srslib_framework/robotics/Pose.hpp>

#include <srslib_framework/ros/RosTap.hpp>
#include <srslib_framework/ros/message/PoseMessageFactory.hpp>

namespace srs {

class RosTapMsgPose :
    public RosTap
{
public:
    typedef typename Pose<>::BaseType BaseType;

    RosTapMsgPose(string topic, string description = "Pose Tap") :
        RosTap(topic, description),
        currentPose_(Pose<>::INVALID)
    {
        RosTap::reset();
    }

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
        set(Pose<>::INVALID);
    }

    void set(Pose<> newPose)
    {
        currentPose_ = newPose;
        setNewData(true);
    }

protected:
    bool connect()
    {
        rosSubscriber_ = rosNodeHandle_.subscribe(getTopic(), 10, &RosTapMsgPose::onPose, this);
        return true;
    }

private:
    void onPose(const srslib_framework::MsgPoseConstPtr message)
    {
        set(PoseMessageFactory::msg2Pose(message));
    }

    Pose<> currentPose_;
};

} // namespace srs

#endif // ROSTAPMSGPOSE_HPP_
