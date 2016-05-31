/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef VELOCITY_HPP_
#define VELOCITY_HPP_

#include <string>
#include <sstream>
using namespace std;

#include <opencv2/opencv.hpp>

#include <srslib_framework/platform/Object.hpp>

namespace srs {

template<typename TYPE = double>
struct Velocity : public Object
{
    typedef TYPE BaseType;

    Velocity(double arrivalTime, BaseType linear, BaseType angular) :
        arrivalTime(arrivalTime),
        linear(linear),
        angular(angular)
    {}

    Velocity(BaseType linear = BaseType(), BaseType angular = BaseType()) :
        arrivalTime(0.0),
        linear(linear),
        angular(angular)
    {}

    Velocity(const Velocity<BaseType>& other) :
        arrivalTime(other.arrivalTime),
        linear(other.linear),
        angular(other.angular)
    {}

    virtual ~Velocity()
    {}

    friend ostream& operator<<(ostream& stream, const Velocity& velocity)
    {
        stream << "Velocity {at: " << velocity.arrivalTime <<
            ", l: " << velocity.linear <<
            ", a: " << velocity.angular << "}";

        return stream;
    }

    double arrivalTime;
    BaseType linear;
    BaseType angular;
};

} // namespace srs

#endif // VELOCITY_HPP_
