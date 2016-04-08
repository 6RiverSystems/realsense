/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef FILTERSTATE_HPP_
#define FILTERSTATE_HPP_

#include <platform/Object.hpp>
#include <framework/Pose.hpp>

namespace srs {

template<unsigned int TYPE = CV_64F, typename ETYPE = double>
struct FilterState : public Object
{
    enum {
        STATE_X,
        STATE_Y,
        STATE_THETA,
        STATE_V,
        STATE_OMEGA,
        MAX_ENUM
    };

    FilterState(Pose<ETYPE> pose)
    {
        vector = cv::Mat::zeros(1, MAX_ENUM, TYPE);

        vector.at<ETYPE>(STATE_X) = pose.x;
        vector.at<ETYPE>(STATE_Y) = pose.y;
        vector.at<ETYPE>(STATE_THETA) = pose.theta;
    }

    FilterState(const cv::Mat state)
    {
        vector = state;
    }

    FilterState()
    {
        vector = cv::Mat::zeros(1, MAX_ENUM, TYPE);
    }

    ~FilterState()
    {}

    ETYPE getX()
    {
        return vector.at<ETYPE>(STATE_X);
    }

    ETYPE getY()
    {
        return vector.at<ETYPE>(STATE_Y);
    }

    ETYPE getTheta()
    {
        return vector.at<ETYPE>(STATE_THETA);
    }

    ETYPE getV()
    {
        return vector.at<ETYPE>(STATE_V);
    }

    ETYPE getOmega()
    {
        return vector.at<ETYPE>(STATE_OMEGA);
    }

    void setX(ETYPE value)
    {
        vector.at<ETYPE>(STATE_X) = value;
    }

    void setY(ETYPE value)
    {
        vector.at<ETYPE>(STATE_Y) = value;
    }

    void setTheta(ETYPE value)
    {
        vector.at<ETYPE>(STATE_THETA) = value;
    }

    void setV(ETYPE value)
    {
        vector.at<ETYPE>(STATE_V) = value;
    }

    void setOmega(ETYPE value)
    {
        vector.at<ETYPE>(STATE_OMEGA) = value;
    }

    cv::Mat vector;
};

} // namespace srs

#endif // FILTERSTATE_HPP_
