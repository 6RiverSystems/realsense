/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <opencv2/opencv.hpp>

#include <srslib_framework/math/Ocv2Base.hpp>

namespace srs {

template<unsigned int STATE_SIZE, int TYPE = CV_64F>
class Model
{
public:
    typedef typename Ocv2Base<TYPE>::BaseType BaseType;

    Model(BaseType noiseValue = BaseType(1))
    {
        setNoiseMatrix(cv::Mat::eye(STATE_SIZE, STATE_SIZE, TYPE) * noiseValue);
    }

    Model(cv::Mat noiseMatrix)
    {
        setNoiseMatrix(noiseMatrix);
    }

    virtual ~Model()
    {}

protected:
    cv::Mat getNoiseMatrix() const
    {
        return noiseMatrix_;
    }

    void setNoiseMatrix(cv::Mat noiseMatrix)
    {
        noiseMatrix.copyTo(noiseMatrix_);
    }

private:
    cv::Mat noiseMatrix_;
};

} // namespace srs
