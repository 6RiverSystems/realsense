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

class TapDepthCameraPoseOnRobot :
    public SingleDataSource<tf::Transform>
{
public:
    TapDepthCameraPoseOnRobot()
    {}

    virtual ~TapDepthCameraPoseOnRobot()
    {}

    virtual tf::Transform peek() const
    {
        return data_;
    }

    virtual tf::Transform pop()
    {
        updatePose();

        return data_;
    }

    virtual void reset()
    {
        data_ = tf::Transform::getIdentity();
    }

    virtual void set(tf::Transform data)
    {
        // Nothing to do
    }

private:
    void updatePose()
    {
        try
        {
            tf::StampedTransform robotTransform;
            tfListener_.lookupTransform(ChuckTransforms::BASE_FOOTPRINT, ChuckTransforms::DEPTH_CAMERA,
                ros::Time(0), robotTransform);

            data_ = robotTransform;
        }
        catch(const tf::TransformException& e)
        {
            ROS_ERROR_STREAM_THROTTLE_NAMED(1.0, "tap_depth_camera_pose_on_robot", "TF Exception: " << e.what());
        }
    }

    tf::Transform data_;

    tf::TransformListener tfListener_;
};

} // namespace srs
