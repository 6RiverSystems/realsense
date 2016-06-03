/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef POSE_HPP_
#define POSE_HPP_

#include <string>
#include <sstream>
using namespace std;

#include <opencv2/opencv.hpp>

#include <srslib_framework/platform/Object.hpp>
#include <srslib_framework/math/Math.hpp>

namespace srs {

template<typename TYPE = double>
struct Pose : public Object
{
    typedef TYPE BaseType;

    Pose(double arrivalTime, BaseType x, BaseType y, BaseType theta) :
        arrivalTime(arrivalTime),
        x(x),
        y(y),
        theta(theta)
    {}

    Pose(BaseType x = BaseType(), BaseType y = BaseType(), BaseType theta = BaseType()) :
        arrivalTime(0.0),
        x(x),
        y(y),
        theta(theta)
    {}

    Pose(const Pose<BaseType>& other) :
        arrivalTime(other.arrivalTime),
        x(other.x),
        y(other.y),
        theta(other.theta)
    {}

    virtual ~Pose()
    {}

    friend ostream& operator<<(ostream& stream, const Pose& pose)
    {
        stream << "Pose {@: " << pose.arrivalTime <<
            ", x: " << pose.x <<
            ", y: " << pose.y <<
            ", t: " << pose.theta << "}";

        return stream;
    }

    void setThetaDegrees(BaseType deg)
    {
        theta = Math::deg2rad<TYPE>(deg);
    }

    double arrivalTime;
    BaseType x;
    BaseType y;
    BaseType theta;
};

} // namespace srs

#endif // POSE_HPP_
