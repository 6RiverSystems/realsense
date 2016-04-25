/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef POSITIONESTIMATOR_HPP_
#define POSITIONESTIMATOR_HPP_

#include <vector>
using namespace std;

#include <tf/transform_broadcaster.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>

#include <framework/RosTap.hpp>

#include "Configuration.hpp"
#include "Robot.hpp"

#include "PEState.hpp"
#include <filter/ukf/UnscentedKalmanFilter.hpp>

#include <tap/odometry/RosOdometer.hpp>
#include <tap/brain_stem_status/RosBrainStemStatus.hpp>
#include <tap/vel_cmd/RosVelCmd.hpp>

namespace srs {

class PositionEstimator
{
public:
    PositionEstimator();

    ~PositionEstimator()
    {
        disconnectAllTaps();
    }

    void run();

private:
    constexpr static unsigned int REFRESH_RATE_HZ = 50;
    constexpr static double ALPHA = 1.0;
    constexpr static double BETA = 0.0;

    void disconnectAllTaps();

    void publishInformation();

    void scanTapsForData();
    void stepUkf();

    RosBrainStemStatus brainStemStatusTap_;

    VelCmd<> currentCommand_;
    cv::Mat currentCovariance_;
    PEState<> currentState_;
    ros::Time currentTimeStep_;

    RosOdometer odometerTap_;

    ros::Time lastTimeStep_;

    Robot<> robot_;
    ros::NodeHandle rosNodeHandle_;
    ros::Publisher rosPubPose;
    tf::TransformBroadcaster rosTfBroadcaster_;

    RosVelCmd velCmdTap_;

    UnscentedKalmanFilter<STATIC_UKF_STATE_VECTOR_SIZE, STATIC_UKF_COMMAND_VECTOR_SIZE> ukf_;
};

} // namespace srs

#endif  // POSITIONESTIMATOR_HPP_
