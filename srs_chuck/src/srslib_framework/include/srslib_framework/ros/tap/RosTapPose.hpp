/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef ROSTAPMSGPOSE_HPP_
#define ROSTAPMSGPOSE_HPP_

#include <string>
using namespace std;

#include <srslib_framework/Pose.h>

#include <srslib_framework/math/TimeMath.hpp>

#include <srslib_framework/robotics/Pose.hpp>

#include <srslib_framework/ros/tap/RosTap.hpp>
#include <srslib_framework/ros/message/PoseMessageFactory.hpp>

namespace srs {

class RosTapPose :
    public RosTap
{
public:
    typedef typename Pose<>::BaseType BaseType;

    RosTapPose(string topic, string description = "Pose Tap") :
        RosTap(topic, description),
        currentPose_(Pose<>::INVALID)
    {
        RosTap::reset();
    }

    ~RosTapPose()
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
        rosSubscriber_ = rosNodeHandle_.subscribe(getTopic(), 10, &RosTapPose::onPose, this);
        return true;
    }

private:
    void onPose(const srslib_framework::Pose::ConstPtr message)
    {
        set(PoseMessageFactory::msg2Pose(message));
    }

    Pose<> currentPose_;
};

} // namespace srs

#endif // ROSTAPMSGPOSE_HPP_
