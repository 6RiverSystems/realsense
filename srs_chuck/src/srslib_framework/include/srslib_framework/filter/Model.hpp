/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef MODEL_HPP_
#define MODEL_HPP_

#include <opencv2/opencv.hpp>

#include <srslib_framework/math/Ocv2Base.hpp>
#include <srslib_framework/platform/Object.hpp>

namespace srs {

template<unsigned int STATE_SIZE, int TYPE = CV_64F>
class Model : public Object
{
public:
    typedef typename Ocv2Base<TYPE>::BaseType BaseType;

    Model(BaseType noiseValue = BaseType()) :
            Object()
    {
        noiseMatrix_ = cv::Mat::eye(STATE_SIZE, STATE_SIZE, TYPE) * noiseValue;
    }

    Model(cv::Mat noiseMatrix) :
            Object(),
            noiseMatrix_(noiseMatrix)
    {}

    virtual ~Model()
    {}

protected:
    cv::Mat noiseMatrix_;
};

} // namespace srs

#endif // MODEL_HPP_
