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

#include <srsnode_motion/tap/odometry/OdometrySensor.hpp>

#include <srsnode_motion/Robot.hpp>
#include <srsnode_motion/StatePe.hpp>
#include <srsnode_motion/CmdVelocity.hpp>

using namespace srs;

constexpr unsigned int UKF_STATE_SIZE = 5;
constexpr unsigned int UKF_COMMAND_SIZE = 2;

constexpr double DT = 1.0;

#include "data/UkfTestData_11ConstantSteps.hpp"

TEST(Test_Motion, Run11ConstantSteps)
{
    // Create standard robot process model
    Robot<> robot;
    OdometrySensor<UKF_STATE_SIZE> odometer;

    UnscentedKalmanFilter<UKF_STATE_SIZE> ukf(robot);
    ukf.addSensor(&odometer);

    // Prepare a sequence of commands
    vector<const CmdVelocity<>*> commands = {
        &COMMAND_STEP_00,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        nullptr
    };

    // Prepare a sequence of odometry readings
    vector<const Odometry<>*> measurements = {
        &ODOMETRY_STEP_01,
        &ODOMETRY_STEP_01,
        &ODOMETRY_STEP_01,
        &ODOMETRY_STEP_01,
        &ODOMETRY_STEP_01,
        &ODOMETRY_STEP_01,
        &ODOMETRY_STEP_01,
        &ODOMETRY_STEP_01,
        &ODOMETRY_STEP_01,
        &ODOMETRY_STEP_01,
        &ODOMETRY_STEP_00
    };

    // Prepare a sequence of correct states and covariance matrices
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
        STATE_STEP_10
    };
    vector<cv::Mat> correctCovariances = {
        COV_STEP_00,
        COV_STEP_01,
        COV_STEP_02,
        COV_STEP_03,
        COV_STEP_04,
        COV_STEP_05,
        COV_STEP_06,
        COV_STEP_07,
        COV_STEP_08,
        COV_STEP_09,
        COV_STEP_10
    };

    // Prepare the initial state
    Pose<double> pose0;
    pose0.setThetaDegrees(90.0);

    StatePe<> stateT0(pose0);

    cv::Mat covarianceT0 = cv::Mat::eye(UKF_STATE_SIZE, UKF_STATE_SIZE, CV_64F);
    covarianceT0 = covarianceT0 * 0.0001;

    // Reset the Kalman filter
    ukf.reset(stateT0.getVectorForm(), covarianceT0);

    // Go through each step and
    for (unsigned int t = 0; t < measurements.size(); ++t)
    {
        // Push the simulated measurement
        auto odometry = *measurements.at(t);
        odometer.set(t * DT, odometry.velocity.linear, odometry.velocity.angular);

        // Run the step of the UKF
        ukf.run(t * DT, const_cast<CmdVelocity<>*>(commands.at(t)));

        ASSERT_TRUE(test::Compare::similar<>(ukf.getP(), correctCovariances[t], 2e-2)) <<
            " Covariance matrix at time-step " << t;
        ASSERT_TRUE(test::Compare::similar<>(ukf.getX(), correctStates[t], 1e-1)) <<
            " State vector at time-step " << t;
    }
}
