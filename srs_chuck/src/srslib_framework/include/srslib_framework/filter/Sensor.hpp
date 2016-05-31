/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef SENSOR_HPP_
#define SENSOR_HPP_

#include <opencv2/opencv.hpp>

#include <srslib_framework/utils/Ocv2Base.hpp>

#include "Model.hpp"
#include "Measurement.hpp"

namespace srs {

template<unsigned int STATE_SIZE = 5, int TYPE = CV_64F>
class Sensor : public Model<STATE_SIZE, TYPE>
{
public:
    typedef typename Ocv2Base<TYPE>::BaseType BaseType;

    Sensor(BaseType R = BaseType()) :
            Model<STATE_SIZE, TYPE>(R),
            enabled_(true),
            newData_(false)
    {}

    Sensor(cv::Mat noiseMatrix) :
        Model<STATE_SIZE, TYPE>(noiseMatrix),
        enabled_(true),
        newData_(false)
    {}

    virtual ~Sensor()
    {}

    void enable(bool newState)
    {
        enabled_ = newState;
    }

    cv::Mat getR()
    {
        return Model<STATE_SIZE, TYPE>::noiseMatrix_;
    }

    bool isEnabled() const
    {
        return enabled_;
    }

    bool newDataAvailable() const
    {
        return newData_;
    }

    virtual cv::Mat getCurrentData() = 0;
    virtual cv::Mat H(const cv::Mat stateVector) = 0;

protected:
    void setNewData(bool newValue)
    {
        newData_ = newValue;
    }

private:
    bool enabled_;
    bool newData_;
};

} // namespace srs

#endif  // SENSOR_HPP_
