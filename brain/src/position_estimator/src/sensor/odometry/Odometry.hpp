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

template<unsigned int STATE_SIZE = 5, int TYPE = CV_64F>
struct Odometry :
    public Measurement<STATE_SIZE, TYPE>
{
    typedef typename Measurement<STATE_SIZE, TYPE>::BaseType BaseType;

    uint32_t arrivalTime;
    BaseType left;
    BaseType right;

    Odometry(Sensor<STATE_SIZE, TYPE>* sensor,
        uint32_t arrivalTime, BaseType left, BaseType right) :
            Measurement<STATE_SIZE, TYPE>(sensor),
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
