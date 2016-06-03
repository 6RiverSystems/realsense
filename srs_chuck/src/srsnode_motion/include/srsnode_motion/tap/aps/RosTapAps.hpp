/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef ROSAPS_HPP_
#define ROSAPS_HPP_

#include <string>
using namespace std;

#include <ros/ros.h>

#include <srslib_framework/math/Time.hpp>
#include <srsnode_motion/tap/aps/ApsSensor.hpp>
#include <srslib_framework/ros/RosTap.hpp>

#include <geometry_msgs/PoseStamped.h>

namespace srs {

class RosTapAps :
    public RosTap
{
public:
    typedef typename ApsSensor<STATIC_UKF_STATE_VECTOR_SIZE, STATIC_UKF_CV_TYPE>::BaseType BaseType;

    RosTapAps(ros::NodeHandle rosHandle) :
        RosTap(rosHandle, "Absolute Positioning System Tap")
    {
        sensor_ = new ApsSensor<STATIC_UKF_STATE_VECTOR_SIZE, STATIC_UKF_CV_TYPE>();
    }

    ~RosTapAps()
    {
        disconnectTap();
        delete sensor_;
    }

    Pose<> getPose()
    {
        return sensor_->getPose();
    }

    ApsSensor<STATIC_UKF_STATE_VECTOR_SIZE, STATIC_UKF_CV_TYPE>* getSensor() const
    {
        return sensor_;
    }

    bool newDataAvailable() const
    {
        return sensor_->newDataAvailable();
    }

    void reset()
    {
        sensor_->reset();

        RosTap::reset();
    }

    void set(double arrivalTime, BaseType x, BaseType y, BaseType yaw)
    {
        sensor_->set(arrivalTime, x, y, yaw);

        if (!sensor_->isEnabled())
        {
            ROS_DEBUG_STREAM("APS sensor disabled. Ignoring readings.");
        }

        setNewData(true);
    }

protected:
    bool connect()
    {
        rosSubscriber_ = rosNodeHandle_.subscribe("/sensors/aps/raw", 100,
            &RosTapAps::onAps, this);

        return true;
    }

private:
    void onAps(const geometry_msgs::PoseStampedPtr& message)
    {
        set(Time::time2number(message->header.stamp),
            static_cast<double>(message->pose.position.x),
            static_cast<double>(message->pose.position.y),
            static_cast<double>(tf::getYaw(message->pose.orientation)));
    }

    ApsSensor<STATIC_UKF_STATE_VECTOR_SIZE, STATIC_UKF_CV_TYPE>* sensor_;
};

} // namespace srs

#endif // ROSAPS_HPP_
