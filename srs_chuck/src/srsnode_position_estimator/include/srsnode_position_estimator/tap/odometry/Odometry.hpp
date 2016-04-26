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

namespace srs {

template<typename TYPE = double>
struct Odometry : public Measurement
{
    Odometry(uint32_t arrivalTime, TYPE linear, TYPE angular) :
            arrivalTime(arrivalTime),
            linear(linear),
            angular(angular)
    {}

    virtual ~Odometry()
    {}

    string toString()
    {
        ostringstream output;
        output << "Odometry {";
        output << "  arrivalTime: " << arrivalTime << endl;
        output << "       linear: " << linear << endl;
        output << "      angular: " << angular << endl;
        output << "}" << endl;

        return output.str();
    }

    uint32_t arrivalTime;
    TYPE linear;
    TYPE angular;
};

} // namespace srs

#endif // ODOMETRY_HPP_
