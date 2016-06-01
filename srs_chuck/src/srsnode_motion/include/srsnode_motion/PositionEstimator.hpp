/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef POSITIONESTIMATOR_HPP_
#define POSITIONESTIMATOR_HPP_

#include <string>
#include <vector>
using namespace std;

#include <srslib_framework/filter/ukf/UnscentedKalmanFilter.hpp>
#include <srslib_framework/filter/Sensor.hpp>
#include <srslib_framework/robotics/Pose.hpp>
#include <srslib_framework/robotics/Velocity.hpp>
#include <srslib_framework/ros/RosTap.hpp>

#include <srsnode_motion/Configuration.hpp>
#include <srsnode_motion/Robot.hpp>
#include <srsnode_motion/StatePe.hpp>

#include <srsnode_motion/tap/odometry/RosTapOdometry.hpp>
#include <srsnode_motion/tap/brain_stem_status/RosTapBrainStemStatus.hpp>
#include <srsnode_motion/tap/initial_pose/RosTapInitialPose.hpp>

namespace srs {

class PositionEstimator
{
public:
    PositionEstimator() :
        ukf_(robot_, ALPHA, BETA),
        previousTime_(-1.0)
    {}

    ~PositionEstimator()
    {}

    void addSensor(Sensor<>* sensor)
    {
        ukf_.addSensor(sensor);
    }

    Pose<> getPose()
    {
        StatePe<> currentState = StatePe<>(ukf_.getX());
        return currentState.getPose();
    }

    Velocity<> getVelocity()
    {
        StatePe<> currentState = StatePe<>(ukf_.getX());
        return currentState.getVelocity();
    }

    void reset(Pose<> initialPose)
    {
        StatePe<> currentState = StatePe<>(initialPose);
        cv::Mat currentCovariance = robot_.getQ();

        ukf_.reset(currentState.getVectorForm(), currentCovariance);
    }

    void run(Odometry<> odometry)
    {
        ROS_INFO_STREAM_NAMED("PositionEstimator", "Position Estimator Odometry: " << odometry);

        // #######
        double currentTime = odometry.velocity.arrivalTime;
        double dT = currentTime - previousTime_;
        if (previousTime_ < 0.0)
        {
            dT = 1.0 / 50.0;
        }
        previousTime_ = currentTime;

        // Transform a velocity point into a command
        CmdVelocity<> command = CmdVelocity<>(odometry.velocity);

        // Advance the state of the UKF
        ukf_.run(dT, &command);
    }

private:
    constexpr static double ALPHA = 0.5;
    constexpr static double BETA = 2.0;

    Robot<> robot_;

    double previousTime_;

    UnscentedKalmanFilter<STATIC_UKF_STATE_VECTOR_SIZE, STATIC_UKF_COMMAND_VECTOR_SIZE> ukf_;
};

} // namespace srs

#endif  // POSITIONESTIMATOR_HPP_
