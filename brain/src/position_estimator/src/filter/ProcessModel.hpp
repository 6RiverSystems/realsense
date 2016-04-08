/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef PROCESSMODEL_HPP_
#define PROCESSMODEL_HPP_

#include <opencv2/opencv.hpp>

#include <platform/Object.hpp>

#include "FilterState.hpp"
#include "Command.hpp"

namespace srs {

template<unsigned int TYPE = CV_64F, typename ETYPE = double>
class ProcessModel : public Object
{
public:
    ProcessModel() :
        Object()
    {}

    virtual ~ProcessModel()
    {}

    virtual cv::Mat transform(
        const cv::Mat state,
        Command<>* const command,
        ETYPE DT) = 0;
};

} // namespace srs

#endif // PROCESSMODEL_HPP_
