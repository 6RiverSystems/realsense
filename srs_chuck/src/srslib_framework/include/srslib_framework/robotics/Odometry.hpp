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

#include <srslib_framework/filter/Measurement.hpp>
#include <srslib_framework/robotics/Velocity.hpp>

namespace srs {

template<typename TYPE = double>
struct Odometry : public Measurement
{
    typedef TYPE BaseType;

    Odometry(double arrivalTime, BaseType linear, BaseType angular) :
        velocity(arrivalTime, linear, angular)
    {}

    Odometry(BaseType linear, BaseType angular) :
        velocity(linear, angular)
    {}

    Odometry(const Odometry<BaseType>& other) :
        velocity(other.velocity)
    {}

    virtual ~Odometry()
    {}

    string toString()
    {
        ostringstream output;
        output << "Odometry {";
        output << "  velocity: " << velocity.toString() << endl;
        output << "}" << endl;

        return output.str();
    }

    Velocity<TYPE> velocity;
};

} // namespace srs

#endif // ODOMETRY_HPP_
