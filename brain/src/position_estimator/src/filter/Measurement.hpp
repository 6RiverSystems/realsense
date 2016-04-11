/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef MEASUREMENT_HPP_
#define MEASUREMENT_HPP_

#include <opencv2/opencv.hpp>

#include <platform/Object.hpp>

namespace srs {

template<unsigned int STATE_SIZE, int TYPE> class Sensor;

template<unsigned int STATE_SIZE = 5, int TYPE = CV_64F>
class Measurement : public Object
{
public:
    Measurement(Sensor<STATE_SIZE, TYPE>* sensor) :
            Object(),
            sensor_(sensor)
    {}

    virtual ~Measurement()
    {}

    Sensor<STATE_SIZE, TYPE>* getSensor()
    {
        return sensor_;
    }

private:
    Sensor<STATE_SIZE, TYPE>* sensor_;
};

} // namespace srs

#endif // MEASUREMENT_HPP_
