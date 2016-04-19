/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef POSITIONESTIMATOR_HPP_
#define POSITIONESTIMATOR_HPP_

#include <vector>
using namespace std;

#include <geometry_msgs/Twist.h>

#include <framework/RosSensor.hpp>

#include "Robot.hpp"
#include "PEState.hpp"
#include "VelCmd.hpp"

#include <filter/ukf/UnscentedKalmanFilter.hpp>

#include "Configuration.hpp"

namespace srs {

class PositionEstimator
{
public:
    PositionEstimator();
    ~PositionEstimator();

    void addSensor(const RosSensor* newSensor);

    void run();

private:
    constexpr static unsigned int REFRESH_RATE_HZ = 1;
    constexpr static double ALPHA = 1.0;
    constexpr static double BETA = 0.0;

    void cbCmdVelReceived(geometry_msgs::TwistConstPtr message);

    Robot<> robot_;
    vector<const RosSensor*> sensors_;

    UnscentedKalmanFilter<STATIC_UKF_STATE_VECTOR_SIZE, STATIC_UKF_COMMAND_VECTOR_SIZE> ukf_;

    PEState<> currentState_;
    cv::Mat currentCovariance_;
    VelCmd<>* currentCommand_;

    ros::NodeHandle rosNodeHandle_;
    ros::Subscriber rosSubscriberCmdVel_;
};

} // namespace srs

#endif  // POSITIONESTIMATOR_HPP_
