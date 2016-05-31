/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef ROSTAPINITIALPOSE_HPP_
#define ROSTAPINITIALPOSE_HPP_

#include <string>
using namespace std;

#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <srslib_framework/ros/RosTap.hpp>
#include <srslib_framework/robotics/Pose.hpp>
#include <srslib_framework/math/Time.hpp>

namespace srs {

class RosTapInitialPose :
    public RosTap
{
public:
    typedef typename Pose<>::BaseType BaseType;

    RosTapInitialPose(ros::NodeHandle rosHandle) :
        RosTap(rosHandle, "Initial Pose Tap"),
        initialPose_(Pose<>(3.0, 2.0, 0.0))
    {}

    ~RosTapInitialPose()
    {
        disconnectTap();
    }

    Pose<> getPose()
    {
        setNewData(false);
        return initialPose_;
    }

    void reset()
    {
        set(Time::time2number(ros::Time::now()), 3.0, 2.0, 0.0);
    }

    void set(double arrivalTime, BaseType x, BaseType y, BaseType theta)
    {
        initialPose_ = Pose<>(arrivalTime, x, y, theta);
        setNewData(true);
    }

protected:
    bool connect()
    {
        rosSubscriber_ = rosNodeHandle_.subscribe("/srsnode_executive/initial_pose", 1,
            &RosTapInitialPose::onInitialPose, this);

        return true;
    }

private:
    void onInitialPose(geometry_msgs::PoseWithCovarianceStampedConstPtr message)
    {
        set(Time::time2number(message->header.stamp),
            message->pose.pose.position.x,
            message->pose.pose.position.y,
            tf::getYaw(message->pose.pose.orientation));
    }

    Pose<> initialPose_;
};

} // namespace srs

#endif // ROSTAPINITIALPOSE_HPP_
