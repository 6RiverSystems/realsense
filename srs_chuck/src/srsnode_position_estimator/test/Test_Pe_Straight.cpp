/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#include <gtest/gtest.h>

#include <vector>
using namespace std;

#include <srslib_test/utils/Compare.hpp>

#include <srslib_framework/filter/FilterState.hpp>
#include <srslib_framework/filter/Command.hpp>
#include <srslib_framework/filter/Measurement.hpp>
#include <srslib_framework/filter/ukf/UnscentedKalmanFilter.hpp>
#include <srslib_framework/robotics/Pose.hpp>
#include <srslib_framework/robotics/Odometry.hpp>

#include <srsnode_position_estimator/tap/odometry/Odometer.hpp>

#include <srsnode_position_estimator/RobotProfile.hpp>
#include <srsnode_position_estimator/Robot.hpp>
#include <srsnode_position_estimator/StatePe.hpp>
#include <srsnode_position_estimator/CmdVelocity.hpp>

using namespace srs;

constexpr unsigned int UKF_STATE_SIZE = 5;
constexpr unsigned int UKF_COMMAND_SIZE = 2;

constexpr double ALPHA = 1.0;
constexpr double BETA = 0.0;
constexpr double DT = 0.5;

#include "data/UkfTestData_Straight.hpp"

TEST(Test_pe, Straight)
{
    // Create standard robot process model
    Robot<> robot;
    Odometer<UKF_STATE_SIZE, CV_64F> odometer;

    UnscentedKalmanFilter<UKF_STATE_SIZE> ukf(ALPHA, BETA, robot);
    ukf.addSensor(&odometer);

    // Create a sequence of commands
    vector<const CmdVelocity<>*> commands = {
        &COMMAND_STEP_00,
        &COMMAND_STEP_01,
        &COMMAND_STEP_02,
        &COMMAND_STEP_03,
        &COMMAND_STEP_04,
        &COMMAND_STEP_05,
        &COMMAND_STEP_06,
        &COMMAND_STEP_07,
        &COMMAND_STEP_08,
        &COMMAND_STEP_09,
        &COMMAND_STEP_10,
        &COMMAND_STEP_11,
        &COMMAND_STEP_12,
        &COMMAND_STEP_13
    };

    // Create the sequence of measurements
    vector<const Odometry<>*> measurements = {
        &ODOMETRY_STEP_00,
        &ODOMETRY_STEP_01,
        &ODOMETRY_STEP_02,
        &ODOMETRY_STEP_03,
        &ODOMETRY_STEP_04,
        &ODOMETRY_STEP_05,
        &ODOMETRY_STEP_06,
        &ODOMETRY_STEP_07,
        &ODOMETRY_STEP_08,
        &ODOMETRY_STEP_09,
        &ODOMETRY_STEP_10,
        &ODOMETRY_STEP_11,
        &ODOMETRY_STEP_12,
        &ODOMETRY_STEP_13
    };

    vector<cv::Mat> correctStates = {
        STATE_STEP_00,
        STATE_STEP_01,
        STATE_STEP_02,
        STATE_STEP_03,
        STATE_STEP_04,
        STATE_STEP_05,
        STATE_STEP_06,
        STATE_STEP_07,
        STATE_STEP_08,
        STATE_STEP_09,
        STATE_STEP_10,
        STATE_STEP_11,
        STATE_STEP_12,
        STATE_STEP_13
    };

    // Prepare the initial state
    Pose<double> pose0;
    pose0.setThetaDegrees(90.0);

    StatePe<> stateT0(pose0);
    cv::Mat covarianceT0 = robot.getNoiseMatrix();
    ukf.reset(stateT0.getVectorForm(), covarianceT0);

    for (unsigned int t = 0; t < measurements.size(); ++t)
    {
        // Push the simulated measurement
        auto odometry = *measurements.at(t);
        odometer.set(odometry.arrivalTime, odometry.linear, odometry.angular);

        // Run the step of the UKF
        ukf.run(t * DT, const_cast<CmdVelocity<>*>(commands.at(t)));

        ASSERT_TRUE(test::Compare::similar<>(ukf.getState(), correctStates[t], 1e-1)) <<
            " State vector at time-step " << t;
    }
}
