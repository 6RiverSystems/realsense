/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef ODOMETRYSENSOR_HPP_
#define ODOMETRYSENSOR_HPP_

#include <ros/ros.h>

#include <brain_msgs/RawOdometry.h>

#include <sensor/SensorReadingHandler.hpp>
#include <sensor/SensorFrameQueue.hpp>
#include <filter/Measurement.hpp>

#include <sensor/Sensor.hpp>
#include "Odometry.hpp"

namespace srs {

class OdometrySensor :
    public Sensor
{
public:
    OdometrySensor(const SensorFrameQueue* queue);
    ~OdometrySensor();

    Measurement getCurrentData() const;

    bool newDataAvailable() const;

    void reset();

private:
    void cbMessageReceived(brain_msgs::RawOdometryConstPtr message);

    ros::Subscriber rosSubscriber_;

    Odometry currentData_;
    bool newData_;
};

} // namespace srs

#endif // ODOMETRYSENSOR_HPP_
