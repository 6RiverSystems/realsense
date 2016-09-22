/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef SENSORFRAME_HPP_
#define SENSORFRAME_HPP_

#include <string>
#include <sstream>
using namespace std;

#include <opencv2/opencv.hpp>

#include <srslib_framework/platform/Object.hpp>
#include <srslib_framework/robotics/Velocity.hpp>

namespace srs {

template<typename TYPE = double>
struct SensorFrame : public Object
{
    typedef TYPE BaseType;

    static const SensorFrame<TYPE> INVALID;
    static const SensorFrame<TYPE> ZERO;

    SensorFrame(Velocity<TYPE> odometry, Imu<TYPE> imu) :
        odometry(odometry),
        imu(imu)
    {}

    SensorFrame(const SensorFrame<BaseType>& other) :
        odometry(other.odometry),
        imu(other.imu)
    {}

    virtual ~SensorFrame()
    {}

    friend ostream& operator<<(ostream& stream, const SensorFrame& sensorFrame)
    {
        return stream << "SensorFrame {" <<
            "odometry: " << sensorFrame.odometry <<
            "imu: " << sensorFrame.imu << "}";
    }

    Velocity<TYPE> odometry;
    Imu<TYPE> imu;
};

template<typename TYPE>
const SensorFrame<TYPE> SensorFrame<TYPE>::INVALID = SensorFrame<TYPE>(
    Velocity<TYPE>::INVALID,
    Imu<TYPE>::INVALID
);

template<typename TYPE>
const SensorFrame<TYPE> SensorFrame<TYPE>::ZERO = SensorFrame<TYPE>(
    Velocity<TYPE>::ZERO,
    Imu<TYPE>::ZERO
);

} // namespace srs

#endif // SENSORFRAME_HPP_