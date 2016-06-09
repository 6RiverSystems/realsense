/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef STATEPE_HPP_
#define STATEPE_HPP_

#include <opencv2/opencv.hpp>

#include <srslib_framework/filter/FilterState.hpp>
#include <srslib_framework/math/Ocv2Base.hpp>
#include <srslib_framework/platform/Object.hpp>

#include <srslib_framework/robotics/Pose.hpp>
#include <srslib_framework/robotics/Velocity.hpp>

#include <srsnode_motion/Configuration.hpp>

namespace srs {

template<int TYPE = CV_64F>
struct StatePe : public FilterState<STATIC_UKF_STATE_VECTOR_SIZE, TYPE>
{
    typedef typename FilterState<STATIC_UKF_STATE_VECTOR_SIZE, TYPE>::BaseType BaseType;

    enum {
        STATE_X = 0,
        STATE_Y = 1,
        STATE_THETA = 2,
        STATE_LINEAR = 3,
        STATE_ANGULAR = 4
    };

    StatePe(BaseType x, BaseType y, BaseType theta) :
        FilterState<STATIC_UKF_STATE_VECTOR_SIZE, TYPE>(),
        pose(Pose<BaseType>(x, y, theta)),
        velocity(Velocity<BaseType>())
    {}

    StatePe(Pose<BaseType> pose) :
        FilterState<STATIC_UKF_STATE_VECTOR_SIZE, TYPE>(),
        pose(pose),
        velocity(Velocity<BaseType>())
    {}

    StatePe(Velocity<BaseType> velocity) :
        FilterState<STATIC_UKF_STATE_VECTOR_SIZE, TYPE>(),
        pose(Pose<BaseType>()),
        velocity(velocity)
    {}

    StatePe(cv::Mat vector) :
        FilterState<STATIC_UKF_STATE_VECTOR_SIZE, TYPE>(),
        pose(Pose<BaseType>(
            vector.at<BaseType>(STATE_X),
            vector.at<BaseType>(STATE_Y),
            vector.at<BaseType>(STATE_THETA))),
        velocity(Velocity<BaseType>(
            vector.at<BaseType>(STATE_LINEAR),
            vector.at<BaseType>(STATE_ANGULAR)))
    {}

    StatePe() :
        FilterState<STATIC_UKF_STATE_VECTOR_SIZE, TYPE>(),
        pose(Pose<BaseType>()),
        velocity(Velocity<BaseType>())
    {}

    ~StatePe()
    {}

    Pose<BaseType> getPose()
    {
        return pose;
    }

    cv::Mat getVectorForm()
    {
        cv::Mat state = cv::Mat(STATIC_UKF_STATE_VECTOR_SIZE, 1, TYPE);

        state.at<BaseType>(STATE_X) = pose.x;
        state.at<BaseType>(STATE_Y) = pose.y;
        state.at<BaseType>(STATE_THETA) = pose.theta;
        state.at<BaseType>(STATE_LINEAR) = velocity.linear;
        state.at<BaseType>(STATE_ANGULAR) = velocity.angular;

        return state;
    }

    Velocity<BaseType> getVelocity()
    {
        return velocity;
    }

    friend ostream& operator<<(ostream& stream, const StatePe& statePe)
    {
        return "StatePe {" << statePe.pose << ", " << statePe.velocity << "}";
    }

    Pose<BaseType> pose;
    Velocity<BaseType> velocity;
};

} // namespace srs

#endif // STATEPE_HPP_
