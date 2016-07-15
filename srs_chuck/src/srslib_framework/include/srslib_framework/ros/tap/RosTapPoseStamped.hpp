/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef ROSTAPPOSESTAMPED_HPP_
#define ROSTAPPOSESTAMPED_HPP_

#include <geometry_msgs/PoseStamped.h>

#include <srslib_framework/robotics/Pose.hpp>
#include <srslib_framework/ros/RosTap.hpp>
#include <srslib_framework/ros/message/PoseMessageFactory.hpp>

namespace srs {

class RosTapPoseStamped :
    public RosTap
{
public:
    typedef typename Pose<>::BaseType BaseType;

    RosTapPoseStamped(string topic, string description = "Pose Stamped Tap") :
        RosTap(topic, description),
        currentPose_(Pose<>::INVALID)
    {
        RosTap::reset();
    }

    ~RosTapPoseStamped()
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
        rosSubscriber_ = rosNodeHandle_.subscribe(getTopic(), 10, &RosTapPoseStamped::onPose, this);
        return true;
    }

private:
    void onPose(const geometry_msgs::PoseStampedConstPtr message)
    {
        set(PoseMessageFactory::poseStamped2Pose(message));
    }

    Pose<> currentPose_;
};

} // namespace srs

#endif // ROSTAPPOSESTAMPED_HPP_
