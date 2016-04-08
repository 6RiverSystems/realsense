/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef COMMAND_HPP_
#define COMMAND_HPP_

#include <platform/Object.hpp>

namespace srs {

template<unsigned int TYPE = CV_64F, typename ETYPE = double>
struct Command : public Object
{
    enum {
        COMMAND_V,
        COMMAND_OMEGA,
        MAX_ENUM
    };

    Command(const ETYPE v, const ETYPE omega)
    {
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

    ETYPE getV()
    {
        return vector.at<ETYPE>(COMMAND_V);
    }

    ETYPE getOmega()
    {
        return vector.at<ETYPE>(COMMAND_OMEGA);
    }

    void setV(ETYPE value)
    {
        vector.at<ETYPE>(COMMAND_V) = value;
    }

    void setOmega(ETYPE value)
    {
        vector.at<ETYPE>(COMMAND_OMEGA) = value;
    }

    cv::Mat vector;
};

} // namespace srs

#endif // COMMAND_HPP_
