/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef FILTERSTATE_HPP_
#define FILTERSTATE_HPP_

#include <opencv2/opencv.hpp>

#include <platform/Object.hpp>
#include <platform/Ocv2Base.hpp>

#include <framework/Pose.hpp>

namespace srs {

template<int TYPE = CV_64F>
struct FilterState : public Object
{
    typedef typename Ocv2Base<TYPE>::BaseType BaseType;

    enum {
        STATE_X,
        STATE_Y,
        STATE_THETA,
        STATE_V,
        STATE_OMEGA,
        MAX_ENUM
    };

    FilterState(Pose<BaseType> pose)
    {
        vector = cv::Mat::zeros(MAX_ENUM, 1, TYPE);

        vector.at<BaseType>(STATE_X) = pose.x;
        vector.at<BaseType>(STATE_Y) = pose.y;
        vector.at<BaseType>(STATE_THETA) = pose.theta;
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

    BaseType getX()
    {
        return vector.at<BaseType>(STATE_X);
    }

    BaseType getY()
    {
        return vector.at<BaseType>(STATE_Y);
    }

    BaseType getTheta()
    {
        return vector.at<BaseType>(STATE_THETA);
    }

    BaseType getV()
    {
        return vector.at<BaseType>(STATE_V);
    }

    BaseType getOmega()
    {
        return vector.at<BaseType>(STATE_OMEGA);
    }

    void setX(BaseType value)
    {
        vector.at<BaseType>(STATE_X) = value;
    }

    void setY(BaseType value)
    {
        vector.at<BaseType>(STATE_Y) = value;
    }

    void setTheta(BaseType value)
    {
        vector.at<BaseType>(STATE_THETA) = value;
    }

    void setV(BaseType value)
    {
        vector.at<BaseType>(STATE_V) = value;
    }

    void setOmega(BaseType value)
    {
        vector.at<BaseType>(STATE_OMEGA) = value;
    }

    cv::Mat vector;
};

} // namespace srs

#endif // FILTERSTATE_HPP_
