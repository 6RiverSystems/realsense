/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef PROCESSMODEL_HPP_
#define PROCESSMODEL_HPP_

#include <opencv2/opencv.hpp>

#include <platform/Object.hpp>
#include <platform/Ocv2Base.hpp>

#include "Model.hpp"
#include "Command.hpp"

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

    virtual cv::Mat transformWithAB(const cv::Mat stateVector,
        Command<COMMAND_SIZE, TYPE>* const command,
        BaseType dT) = 0;
};

} // namespace srs

#endif // PROCESSMODEL_HPP_
