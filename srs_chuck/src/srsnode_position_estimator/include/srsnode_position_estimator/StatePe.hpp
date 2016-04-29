/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef STATEPE_HPP_
#define STATEPE_HPP_

#include <opencv2/opencv.hpp>

#include <srslib_framework/platform/Object.hpp>
#include <srslib_framework/platform/Ocv2Base.hpp>
#include <srslib_framework/filter/FilterState.hpp>

#include <srslib_framework/robotics/Pose.hpp>
#include <srslib_framework/robotics/Velocity.hpp>

#include <srsnode_position_estimator/Configuration.hpp>

namespace srs {

template<int TYPE = CV_64F>
struct StatePe : public FilterState<STATIC_UKF_STATE_VECTOR_SIZE, TYPE>
{
    typedef typename FilterState<STATIC_UKF_STATE_VECTOR_SIZE, TYPE>::BaseType BaseType;

    enum {
        STATE_X = 0,
        STATE_Y = 1,
        STATE_THETA = 2,
        STATE_V = 3,
        STATE_OMEGA = 4
    };

    StatePe(BaseType x, BaseType y, BaseType theta) :
        FilterState<STATIC_UKF_STATE_VECTOR_SIZE, TYPE>(),
        x(x),
        y(y),
        theta(theta),
        v(BaseType()),
        omega(BaseType())
    {}

    StatePe(Pose<BaseType> pose) :
        FilterState<STATIC_UKF_STATE_VECTOR_SIZE, TYPE>(),
        x(pose.x),
        y(pose.y),
        theta(pose.theta),
        v(BaseType()),
        omega(BaseType())
    {}

    StatePe(Velocity<BaseType> velocity) :
        FilterState<STATIC_UKF_STATE_VECTOR_SIZE, TYPE>(),
        x(BaseType()),
        y(BaseType()),
        theta(BaseType()),
        v(velocity.linear),
        omega(velocity.angular)
    {}

    StatePe(cv::Mat vector) :
        FilterState<STATIC_UKF_STATE_VECTOR_SIZE, TYPE>(),
        x(vector.at<BaseType>(STATE_X)),
        y(vector.at<BaseType>(STATE_Y)),
        theta(vector.at<BaseType>(STATE_THETA)),
        v(vector.at<BaseType>(STATE_V)),
        omega(vector.at<BaseType>(STATE_OMEGA))
    {}

    StatePe() :
        FilterState<STATIC_UKF_STATE_VECTOR_SIZE, TYPE>(),
        x(BaseType()),
        y(BaseType()),
        theta(BaseType()),
        v(BaseType()),
        omega(BaseType())
    {}

    ~StatePe()
    {}

    Pose<BaseType> getPose()
    {
        return Pose<BaseType>(x, y, theta);
    }

    cv::Mat getVectorForm()
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

    string toString()
    {
        ostringstream output;
        output << "StatePe {" << endl;
        output << "      x: " << x << endl;
        output << "      y: " << y << endl;
        output << "  theta: " << theta << endl;
        output << "      v: " << v << endl;
        output << "  omega: " << omega << endl;
        output << "}" << endl;

        return output.str();
    }

    BaseType x;
    BaseType y;
    BaseType theta;
    BaseType v;
    BaseType omega;
};

} // namespace srs

#endif // STATEPE_HPP_
