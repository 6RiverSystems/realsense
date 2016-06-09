/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef PROCESSMODEL_HPP_
#define PROCESSMODEL_HPP_

#include <opencv2/opencv.hpp>

#include <srslib_framework/filter/Command.hpp>
#include <srslib_framework/filter/Model.hpp>
#include <srslib_framework/math/Ocv2Base.hpp>

namespace srs {

template<unsigned int STATE_SIZE = 5, unsigned int COMMAND_SIZE = 2, int TYPE = CV_64F>
class Process : public Model<STATE_SIZE, TYPE>
{
public:
    typedef typename Ocv2Base<TYPE>::BaseType BaseType;

    Process(BaseType noiseValue = BaseType()) :
            Model<STATE_SIZE, TYPE>(noiseValue)
    {}

    Process(cv::Mat noiseMatrix) :
            Model<STATE_SIZE, TYPE>(noiseMatrix)
    {}

    virtual ~Process()
    {}

    cv::Mat getQ()
    {
        return Model<STATE_SIZE, TYPE>::noiseMatrix_;
    }

    virtual cv::Mat FB(
        const cv::Mat stateVector,
        Command<COMMAND_SIZE, TYPE>* const command,
        BaseType dT) = 0;
};

} // namespace srs

#endif // PROCESSMODEL_HPP_
