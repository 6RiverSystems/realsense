/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#ifndef ODOMETRYSENSOR_HPP_
#define ODOMETRYSENSOR_HPP_

#include <ros/ros.h>

#include <framework/SensorReadingHandler.hpp>
#include <framework/SensorFrameQueue.hpp>

#include <sensor/Sensor.hpp>
#include <brain_msgs/RawOdometry.h>

class OdometrySensor :
    public Sensor
{
public:
    OdometrySensor(const SensorFrameQueue* queue);
    ~OdometrySensor();

private:
    void cbMessageReceived(brain_msgs::RawOdometryConstPtr message);

    ros::Subscriber rosSubscriber_;
};

#endif // ODOMETRYSENSOR_HPP_
