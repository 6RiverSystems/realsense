/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef ODOMETRY_HPP_
#define ODOMETRY_HPP_

#include <string>
#include <sstream>
using namespace std;

#include <opencv2/opencv.hpp>

#include <srslib_framework/platform/Object.hpp>
#include <srslib_framework/robotics/Velocity.hpp>

namespace srs {

template<typename TYPE = double>
struct Odometry : public Object
{
    typedef TYPE BaseType;

    static const Odometry<TYPE> INVALID;
    static const Odometry<TYPE> ZERO;

    Odometry(Velocity<TYPE> velocity) :
        velocity(velocity)
    {}

    Odometry(const Odometry<BaseType>& other) :
        velocity(other.velocity)
    {}

    virtual ~Odometry()
    {}

    friend ostream& operator<<(ostream& stream, const Odometry& odometry)
    {
        return stream << "Odometry {" << odometry.velocity << "}";
    }

    Velocity<TYPE> velocity;
};

template<typename TYPE>
const Odometry<TYPE> Odometry<TYPE>::INVALID = Odometry<TYPE>(Velocity<TYPE>::INVALID);

template<typename TYPE>
const Odometry<TYPE> Odometry<TYPE>::ZERO = Odometry<TYPE>(Velocity<TYPE>::ZERO);

} // namespace srs

#endif // ODOMETRY_HPP_
