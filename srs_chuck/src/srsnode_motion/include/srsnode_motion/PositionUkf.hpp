/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef POSITIONUKF_HPP_
#define POSITIONUKF_HPP_

#include <opencv2/opencv.hpp>

#include <srslib_framework/filter/ukf/UnscentedKalmanFilter.hpp>
#include <srslib_framework/robotics/Odometry.hpp>
#include <srslib_framework/robotics/Velocity.hpp>

#include <srsnode_motion/Configuration.hpp>
#include <srsnode_motion/CmdVelocity.hpp>

namespace srs {

class PositionUkf : public UnscentedKalmanFilter<STATIC_UKF_STATE_VECTOR_SIZE, STATIC_UKF_COMMAND_VECTOR_SIZE>
{
public:
    constexpr static double ALPHA = 0.0001;
    constexpr static double BETA = 2.0;

    typedef UnscentedKalmanFilter<STATIC_UKF_STATE_VECTOR_SIZE, STATIC_UKF_COMMAND_VECTOR_SIZE> UkfType;
    typedef typename UkfType::BaseType BaseType;

    PositionUkf(Process<STATIC_UKF_STATE_VECTOR_SIZE, STATIC_UKF_COMMAND_VECTOR_SIZE>& process,
            BaseType alpha = ALPHA, BaseType beta = BETA) :
        UnscentedKalmanFilter(process, alpha, beta)
    {}

    virtual ~PositionUkf()
    {}

    void setTheta(double theta)
    {
        UkfType::x_.at<double>(2) = theta;
    }

    void run(double dT, Odometry<>* odometry)
    {
        if (odometry)
        {
            // Transform a velocity point into a command
            // and advance the state of the UKF
            CmdVelocity<> command = CmdVelocity<>(odometry->velocity);
            UkfType::run(dT, &command);
        }
        else
        {
            UkfType::run(dT, nullptr);
        }
    }

protected:
    cv::Mat addWeighted(const cv::Mat W, const cv::Mat X);

    cv::Mat residual(const cv::Mat A, const cv::Mat B);
};

} // namespace srs

#endif // POSITIONUKF_HPP_
