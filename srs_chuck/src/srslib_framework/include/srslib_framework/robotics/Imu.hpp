/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef IMU_HPP_
#define IMU_HPP_

#include <string>
#include <sstream>
using namespace std;

#include <opencv2/opencv.hpp>

#include <srslib_framework/math/BasicMath.hpp>
#include <srslib_framework/math/AngleMath.hpp>

#include <srslib_framework/platform/Object.hpp>

namespace srs {

template<typename TYPE = double>
struct Imu : public Object
{
    typedef TYPE BaseType;

    static const Imu<TYPE> INVALID;
    static const Imu<TYPE> ZERO;

    Imu(double arrivalTime,
            BaseType yaw, BaseType pitch, BaseType roll,
            BaseType yawRot, BaseType pitchRot, BaseType rollRot) :
        arrivalTime(arrivalTime),
        yaw(yaw),
        pitch(pitch),
        roll(roll),
        yawRot(yawRot),
        pitchRot(pitchRot),
        rollRot(rollRot)
    {}

    Imu(BaseType yaw = BaseType(), BaseType pitch = BaseType(), BaseType roll = BaseType(),
            BaseType yawRot = BaseType(), BaseType pitchRot = BaseType(),
            BaseType rollRot = BaseType()) :
        arrivalTime(0.0),
        yaw(yaw),
        pitch(pitch),
        roll(roll),
        yawRot(yawRot),
        pitchRot(pitchRot),
        rollRot(rollRot)
    {}

    Imu(const Imu<BaseType>& other) :
        arrivalTime(other.arrivalTime),
        yaw(other.yaw),
        pitch(other.pitch),
        roll(other.roll),
        yawRot(other.yawRot),
        pitchRot(other.pitchRot),
        rollRot(other.rollRot)
    {}

    Imu(double arrivalTime, const Imu<BaseType>& other) :
        arrivalTime(arrivalTime),
        yaw(other.yaw),
        pitch(other.pitch),
        roll(other.roll),
        yawRot(other.yawRot),
        pitchRot(other.pitchRot),
        rollRot(other.rollRot)
    {}

    virtual ~Imu()
    {}

    BaseType getYawDegrees()
    {
        return AngleMath::rad2Deg<TYPE>(yaw);
    }

    BaseType getYawRotDegrees()
    {
        return AngleMath::rad2Deg<TYPE>(yawRot);
    }

    bool isValid()
    {
        return !BasicMath::isNan<TYPE>(yaw) && !BasicMath::isNan<TYPE>(yawRot) &&
            !BasicMath::isNan<TYPE>(pitch) && !BasicMath::isNan<TYPE>(pitchRot) &&
            !BasicMath::isNan<TYPE>(roll) && !BasicMath::isNan<TYPE>(rollRot);
    }

    friend ostream& operator<<(ostream& stream, const Imu& imu)
    {
        stream << "Imu {@: " << imu.arrivalTime <<
            ", yaw: " << imu.yaw <<
            ", pitch: " << imu.pitch <<
            ", roll: " << imu.roll <<
            ", yawROT: " << imu.yawRot <<
            ", pitchROT: " << imu.pitchRot <<
            ", rollROT: " << imu.rollRot << "}";

        return stream;
    }

    void setYawDegrees(BaseType deg)
    {
        yaw = AngleMath::deg2Rad<TYPE>(deg);
    }

    void setYawRotDegrees(BaseType deg)
    {
        yawRot = AngleMath::deg2Rad<TYPE>(deg);
    }

    double arrivalTime;
    BaseType yaw;
    BaseType pitch;
    BaseType roll;
    BaseType yawRot;
    BaseType pitchRot;
    BaseType rollRot;
};

template<typename TYPE>
const Imu<TYPE> Imu<TYPE>::INVALID = Imu<TYPE>(
    numeric_limits<TYPE>::quiet_NaN(),
    numeric_limits<TYPE>::quiet_NaN(),
    numeric_limits<TYPE>::quiet_NaN(),
    numeric_limits<TYPE>::quiet_NaN(),
    numeric_limits<TYPE>::quiet_NaN(),
    numeric_limits<TYPE>::quiet_NaN()
);

template<typename TYPE>
const Imu<TYPE> Imu<TYPE>::ZERO = Imu<TYPE>(TYPE(), TYPE(), TYPE(), TYPE(), TYPE(), TYPE());

} // namespace srs

#endif // IMU_HPP_
