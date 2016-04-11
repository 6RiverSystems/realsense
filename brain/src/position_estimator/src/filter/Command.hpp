/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef COMMAND_HPP_
#define COMMAND_HPP_

#include <opencv2/opencv.hpp>

#include <platform/Object.hpp>
#include <platform/Ocv2Base.hpp>

namespace srs {

template<int TYPE = CV_64F>
struct Command : public Object
{
    typedef typename Ocv2Base<TYPE>::BaseType BaseType;

    enum {
        COMMAND_V,
        COMMAND_OMEGA,
        MAX_ENUM
    };

    Command(const BaseType v, const BaseType omega)
    {
        vector = cv::Mat::zeros(1, MAX_ENUM, TYPE);

        setV(v);
        setOmega(omega);
    }

    Command(const cv::Mat command)
    {
        vector = command;
    }

    Command()
    {
        vector = cv::Mat::zeros(1, MAX_ENUM, TYPE);
    }

    ~Command()
    {}

    BaseType getV()
    {
        return vector.at<BaseType>(COMMAND_V);
    }

    BaseType getOmega()
    {
        return vector.at<BaseType>(COMMAND_OMEGA);
    }

    void setV(BaseType value)
    {
        vector.at<BaseType>(COMMAND_V) = value;
    }

    void setOmega(BaseType value)
    {
        vector.at<BaseType>(COMMAND_OMEGA) = value;
    }

    cv::Mat vector;
};

} // namespace srs

#endif // COMMAND_HPP_
