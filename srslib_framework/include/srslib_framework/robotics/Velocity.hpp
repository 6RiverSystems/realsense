/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
#include <sstream>
using namespace std;

#include <srslib_framework/math/BasicMath.hpp>
#include <srslib_framework/math/AngleMath.hpp>

namespace srs {

template<typename TYPE = double>
struct Velocity
{
    typedef TYPE BaseType;

    static const Velocity<TYPE> INVALID;
    static const Velocity<TYPE> ZERO;

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

    Velocity(double arrivalTime, const Velocity<BaseType>& other) :
        arrivalTime(arrivalTime),
        linear(other.linear),
        angular(other.angular)
    {}

    virtual ~Velocity()
    {}

    bool isValid()
    {
        return !BasicMath::isNan<TYPE>(linear) && !BasicMath::isNan<TYPE>(angular);
    }

    friend ostream& operator<<(ostream& stream, const Velocity& velocity)
    {
        stream << "Velocity {@: " << velocity.arrivalTime <<
            ", l: " << velocity.linear <<
            ", a: " << velocity.angular << "}";

        return stream;
    }

    double arrivalTime;
    BaseType linear;
    BaseType angular;
};

template<typename TYPE>
const Velocity<TYPE> Velocity<TYPE>::INVALID = Velocity<TYPE>(
    numeric_limits<TYPE>::quiet_NaN(),
    numeric_limits<TYPE>::quiet_NaN()
);

template<typename TYPE>
const Velocity<TYPE> Velocity<TYPE>::ZERO = Velocity<TYPE>(TYPE(), TYPE());

} // namespace srs
