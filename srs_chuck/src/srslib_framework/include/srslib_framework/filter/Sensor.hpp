/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef SENSOR_HPP_
#define SENSOR_HPP_

#include <opencv2/opencv.hpp>

#include <srslib_framework/filter/Measurement.hpp>
#include <srslib_framework/filter/Model.hpp>
#include <srslib_framework/math/Ocv2Base.hpp>

namespace srs {

template<unsigned int STATE_SIZE = 5, int TYPE = CV_64F>
class Sensor : public Model<STATE_SIZE, TYPE>
{
public:
    typedef typename Ocv2Base<TYPE>::BaseType BaseType;

    Sensor(BaseType R = BaseType(1)) :
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

    virtual cv::Mat getCurrentData() = 0;

    cv::Mat getR() const
    {
        return Model<STATE_SIZE, TYPE>::getNoiseMatrix();
    }

    virtual cv::Mat H(const cv::Mat stateVector) = 0;

    bool isEnabled() const
    {
        return enabled_;
    }

    bool newDataAvailable() const
    {
        return newData_;
    }

    void setR(cv::Mat R)
    {
        Model<STATE_SIZE, TYPE>::setNoiseMatrix(R);
    }

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
