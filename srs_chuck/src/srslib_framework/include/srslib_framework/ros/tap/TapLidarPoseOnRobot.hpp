/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <srslib_framework/robotics/Pose.hpp>
#include <srslib_framework/ros/tap/subscriber/SingleDataSource.hpp>
#include <srslib_framework/ros/topics/ChuckTransforms.hpp>

namespace srs {

class TapLidarPoseOnRobot :
    public SingleDataSource<Pose<>>
{
public:
    TapLidarPoseOnRobot()
    {}

    virtual ~TapLidarPoseOnRobot()
    {}

    virtual Pose<> peek() const
    {
        return data_;
    }

    virtual Pose<> pop()
    {
        updatePose();

        return data_;
    }

    virtual void reset()
    {
        data_ = Pose<>::INVALID;
    }

    virtual void set(Pose<> data)
    {
        // Nothing to do
    }

private:
    void updatePose()
    {
        try
        {
            tf::StampedTransform robotTransform;
            tfListener_.lookupTransform(ChuckTransforms::BASE_FOOTPRINT, ChuckTransforms::LIDAR,
                ros::Time(0), robotTransform);

            tf::Vector3 location = robotTransform.getOrigin();
            tf::Quaternion orientation = robotTransform.getRotation();

            data_ = Pose<>(location.getX(), location.getY(), tf::getYaw(orientation));

            ROS_DEBUG_STREAM_THROTTLE_NAMED(1.0, "tap_robot_pose", "Robot pose" << data_);
        }
        catch(const tf::TransformException& e)
        {
            ROS_ERROR_STREAM_THROTTLE_NAMED(1.0, "tap_robot_pose", "TF Exception: " << e.what());
        }
    }

    Pose<> data_;

    tf::TransformListener tfListener_;
};

} // namespace srs
