/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef ROSTAPPOSESTAMPED_HPP_
#define ROSTAPPOSESTAMPED_HPP_

#include <string>
using namespace std;

#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>

#include <srslib_framework/math/TimeMath.hpp>
#include <srslib_framework/robotics/Pose.hpp>
#include <srslib_framework/ros/RosTap.hpp>

namespace srs {

class RosTapPoseStamped :
    public RosTap
{
public:
    typedef typename Pose<>::BaseType BaseType;

    RosTapPoseStamped(string topic, string description = "Pose Stamped Tap") :
        RosTap(topic, description)
    {}

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
        rosSubscriber_ = rosNodeHandle_.subscribe(getTopic(), 10, &RosTapPoseStamped::onPose, this);
        return true;
    }

private:
    void onPose(const geometry_msgs::PoseStampedConstPtr message)
    {
        set(TimeMath::time2number(message->header.stamp),
            message->pose.position.x,
            message->pose.position.y,
            tf::getYaw(message->pose.orientation));
    }

    Pose<> currentPose_;
};

} // namespace srs

#endif // ROSTAPPOSESTAMPED_HPP_
