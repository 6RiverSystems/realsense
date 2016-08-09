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

#include <srslib_framework/math/TimeMath.hpp>
#include <srslib_framework/ros/RosTap.hpp>
#include <srsnode_motion/tap/sensor_frame/aps/ApsSensor.hpp>

#include <srslib_framework/MsgPose.h>

namespace srs {

class RosTapAps :
    public RosTap
{
public:
    typedef typename ApsSensor<STATIC_UKF_STATE_VECTOR_SIZE, STATIC_UKF_CV_TYPE>::BaseType BaseType;

    RosTapAps() :
        RosTap("Absolute Positioning System Tap"),
        thetaPoints_(0),
        thetaSumCosine_(0.0),
        thetaSumSine_(0.0)
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

        thetaSumCosine_ = 0.0;
        thetaSumSine_ = 0.0;
        thetaPoints_ = 0;

        RosTap::reset();
    }

    void set(Pose<> newAps)
    {
        sensor_->set(newAps);

        if (!sensor_->isEnabled())
        {
            ROS_DEBUG_STREAM_NAMED("ros_tap_aps", "APS disabled. Ignoring readings.");
        }

        setNewData(sensor_->newDataAvailable());
    }

protected:
    bool connect()
    {
        rosSubscriber_ = rosNodeHandle_.subscribe("/internal/sensors/aps/raw", 100,
            &RosTapAps::onAps, this);

        return true;
    }

private:
    void onAps(const srslib_framework::MsgPoseConstPtr& message)
    {
        // If the tap hasn't heard from the physical sensor
        // for more than 0.5 seconds, it's likely that the
        // average values (sine and cosine) are not valid anymore.
        // Reset the sums for the average
        ros::Time currentTime = ros::Time::now();
        if (TimeMath::isTimeElapsed(1.0, previousApsTime_, currentTime))
        {
            reset();
        }
        previousApsTime_ = currentTime;

        double theta = AngleMath::deg2rad(message->theta);

        thetaSumCosine_ += cos(theta);
        thetaSumSine_ += sin(theta);
        thetaPoints_ += 1;
        if (!thetaPoints_)
        {
            thetaPoints_ = 1;
        }

        double averageAngle = AngleMath::normalizeAngleRad<double>(atan2(
            thetaSumSine_ / thetaPoints_,
            thetaSumCosine_ / thetaPoints_));

        set(Pose<>(TimeMath::time2number(message->header.stamp),
            static_cast<double>(message->x),
            static_cast<double>(message->y),
            static_cast<double>(averageAngle)));
    }

    ApsSensor<STATIC_UKF_STATE_VECTOR_SIZE, STATIC_UKF_CV_TYPE>* sensor_;

    unsigned int thetaPoints_;
    double thetaSumCosine_;
    double thetaSumSine_;

    ros::Time previousApsTime_;
};

} // namespace srs

#endif // ROSAPS_HPP_
