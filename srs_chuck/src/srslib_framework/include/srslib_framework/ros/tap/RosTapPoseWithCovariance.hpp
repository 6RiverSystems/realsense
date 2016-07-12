/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef ROSTAPPOSEWITHCOVARIANCE_HPP_
#define ROSTAPPOSEWITHCOVARIANCE_HPP_

#include <string>
using namespace std;

#include <tf/tf.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <srslib_framework/math/TimeMath.hpp>
#include <srslib_framework/robotics/Pose.hpp>
#include <srslib_framework/ros/RosTap.hpp>

namespace srs {

class RosTapPoseWithCovariance :
    public RosTap
{
public:
    typedef typename Pose<>::BaseType BaseType;

    RosTapPoseWithCovariance(string topic, string description = "Pose With Covariance Tap") :
        RosTap(topic, description),
        currentPose_(Pose<>::INVALID)
    {
        RosTap::reset();
    }

    ~RosTapPoseWithCovariance()
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
        rosSubscriber_ = rosNodeHandle_.subscribe(getTopic(), 10,
            &RosTapPoseWithCovariance::onPose, this);
        return true;
    }

private:
    void onPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr message)
    {
        set(TimeMath::time2number(message->header.stamp),
            message->pose.pose.position.x,
            message->pose.pose.position.y,
            tf::getYaw(message->pose.pose.orientation));
    }

    Pose<> currentPose_;
};

} // namespace srs

#endif // ROSTAPPOSEWITHCOVARIANCE_HPP_
