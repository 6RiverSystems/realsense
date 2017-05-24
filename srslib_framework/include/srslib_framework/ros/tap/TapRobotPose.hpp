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
#include <srslib_framework/ros/message/PoseMessageFactory.hpp>
#include <srslib_framework/ros/tap/subscriber/SingleDataSource.hpp>
#include <srslib_framework/chuck/ChuckTransforms.hpp>

namespace srs {

class TapRobotPose :
    public SingleDataSource<Pose<>>
{
public:
    TapRobotPose()
    {}

    virtual ~TapRobotPose()
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
            if (tfListener_.canTransform(ChuckTransforms::MAP, ChuckTransforms::BASE_FOOTPRINT,
                ros::Time(0))) {
                tfListener_.lookupTransform(ChuckTransforms::MAP, ChuckTransforms::BASE_FOOTPRINT,
                    ros::Time(0), robotTransform);

                data_ = PoseMessageFactory::transform2Pose(robotTransform);
            } else {
                data_ = Pose<>::INVALID;
            }

            ROS_DEBUG_STREAM_THROTTLE_NAMED(1.0, "tap_robot_pose", "Robot pose" << data_);
        }
        catch(const tf::TransformException& e)
        {
            ROS_DEBUG_STREAM_THROTTLE_NAMED(1.0, "tap_robot_pose", "TF Exception: " << e.what());
        }
    }

    Pose<> data_;

    tf::TransformListener tfListener_;
};

} // namespace srs
