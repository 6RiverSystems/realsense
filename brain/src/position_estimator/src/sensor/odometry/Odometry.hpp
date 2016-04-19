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

#include <filter/Measurement.hpp>

namespace srs {

template<typename TYPE = double>
struct Odometry : public Measurement
{
    uint32_t arrivalTime;
    TYPE left;
    TYPE right;

    Odometry(uint32_t arrivalTime, TYPE left, TYPE right) :
            arrivalTime(arrivalTime),
            left(left),
            right(right)
    {}

    virtual ~Odometry()
    {}

    string toString()
    {
        ostringstream output;
        output << "Odometry ";
        output << "arrivalTime: " << arrivalTime << endl;
        output << "left: " << left << endl;
        output << "right: " << right << endl;

        return output.str();
    }
};

} // namespace srs

#endif // ODOMETRY_HPP_
