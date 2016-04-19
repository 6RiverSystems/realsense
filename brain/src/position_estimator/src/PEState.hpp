/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef POSITIONESTIMATORSTATE_HPP_
#define POSITIONESTIMATORSTATE_HPP_

#include <opencv2/opencv.hpp>

#include <platform/Object.hpp>
#include <platform/Ocv2Base.hpp>

#include <filter/FilterState.hpp>

#include <framework/Pose.hpp>
#include <framework/Velocity.hpp>

#include "Configuration.hpp"

namespace srs {

template<int TYPE = CV_64F>
struct PEState : public FilterState<STATIC_UKF_STATE_VECTOR_SIZE, TYPE>
{
    typedef typename FilterState<STATIC_UKF_STATE_VECTOR_SIZE, TYPE>::BaseType BaseType;

    enum {
        STATE_X,
        STATE_Y,
        STATE_THETA,
        STATE_V,
        STATE_OMEGA,
    };

    PEState(BaseType x, BaseType y, BaseType theta) :
        FilterState<STATIC_UKF_STATE_VECTOR_SIZE, TYPE>(),
        x(x),
        y(y),
        theta(theta),
        v(BaseType()),
        omega(BaseType())
    {}

    PEState(Pose<BaseType> pose) :
        FilterState<STATIC_UKF_STATE_VECTOR_SIZE, TYPE>(),
        x(pose.x),
        y(pose.y),
        theta(pose.theta),
        v(BaseType()),
        omega(BaseType())
    {}

    PEState(Velocity<BaseType> velocity) :
        FilterState<STATIC_UKF_STATE_VECTOR_SIZE, TYPE>(),
        x(BaseType()),
        y(BaseType()),
        theta(BaseType()),
        v(velocity.linear),
        omega(velocity.angular)
    {}

    PEState(cv::Mat vector) :
        FilterState<STATIC_UKF_STATE_VECTOR_SIZE, TYPE>(),
        x(vector.at<BaseType>(STATE_X)),
        y(vector.at<BaseType>(STATE_Y)),
        theta(vector.at<BaseType>(STATE_THETA)),
        v(vector.at<BaseType>(STATE_V)),
        omega(vector.at<BaseType>(STATE_OMEGA))
    {}

    PEState() :
        FilterState<STATIC_UKF_STATE_VECTOR_SIZE, TYPE>(),
        x(BaseType()),
        y(BaseType()),
        theta(BaseType()),
        v(BaseType()),
        omega(BaseType())
    {}

    ~PEState()
    {}

    Pose<BaseType> getPose()
    {
        return Pose<BaseType>(x, y, theta);
    }

    cv::Mat getStateVector()
    {
        cv::Mat state = cv::Mat(STATIC_UKF_STATE_VECTOR_SIZE, 1, TYPE);

        state.at<BaseType>(STATE_X) = x;
        state.at<BaseType>(STATE_Y) = y;
        state.at<BaseType>(STATE_THETA) = theta;
        state.at<BaseType>(STATE_V) = v;
        state.at<BaseType>(STATE_OMEGA) = omega;

        return state;
    }

    Velocity<BaseType> getVelocity()
    {
        return Velocity<BaseType>(v, omega);
    }

    BaseType x;
    BaseType y;
    BaseType theta;
    BaseType v;
    BaseType omega;
};

} // namespace srs

#endif // POSITIONESTIMATORSTATE_HPP_
