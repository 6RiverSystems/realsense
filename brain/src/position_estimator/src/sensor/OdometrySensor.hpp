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
#include <framework/SensorReading.hpp>

#include <sensor/Sensor.hpp>
#include <brain_msgs/RawOdometry.h>

namespace srs {

struct OdometryReading :
    public SensorReading
{
    uint32_t arrivalTime;
    uint16_t left;
    uint16_t right;

    OdometryReading(uint32_t arrivalTime, uint16_t left, uint16_t right) :
        arrivalTime(arrivalTime),
        left(left),
        right(right)
    {}
};

class OdometrySensor :
    public Sensor
{
public:
    OdometrySensor(const SensorFrameQueue* queue);
    ~OdometrySensor();

    OdometryReading getCurrentData() const
    {
        return OdometryReading(currentData_);
    }

    bool newDataAvailable() const;
    void reset();

private:
    void cbMessageReceived(brain_msgs::RawOdometryConstPtr message);

    ros::Subscriber rosSubscriber_;

    OdometryReading currentData_;
    bool newData_;
};

} // namespace srs

#endif // ODOMETRYSENSOR_HPP_
