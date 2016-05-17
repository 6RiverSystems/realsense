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
        ukf_(ALPHA, BETA, robot_)
    {}

    ~PositionEstimator()
    {}

    void addSensor(Sensor<>* sensor)
    {
        ukf_.addSensor(sensor);
    }

    Pose<> getPose()
    {
        StatePe<> currentState = StatePe<>(ukf_.getState());
        return currentState.getPose();
    }

    Velocity<> getVelocity()
    {
        StatePe<> currentState = StatePe<>(ukf_.getState());
        return currentState.getVelocity();
    }

    void reset(Pose<> initialPose)
    {
        StatePe<> currentState = StatePe<>(initialPose);
        cv::Mat currentCovariance = robot_.getNoiseMatrix();

        ukf_.reset(currentState.getVectorForm(), currentCovariance);
    }

    void run(double dT, Velocity<>* commandVelocity)
    {
        CmdVelocity<> command;
        if (commandVelocity)
        {
            command = CmdVelocity<>(*commandVelocity);
        }

        // Advance the state of the UKF
        ukf_.run(dT, commandVelocity ? &command : nullptr);
    }

private:
    constexpr static double ALPHA = 1.0;
    constexpr static double BETA = 0.0;

    Robot<> robot_;

    UnscentedKalmanFilter<STATIC_UKF_STATE_VECTOR_SIZE, STATIC_UKF_COMMAND_VECTOR_SIZE> ukf_;
};

} // namespace srs

#endif  // POSITIONESTIMATOR_HPP_
