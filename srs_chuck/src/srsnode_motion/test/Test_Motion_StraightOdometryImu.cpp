/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#include <gtest/gtest.h>

#include <vector>
using namespace std;

#include <srslib_test/utils/Compare.hpp>
#include <srslib_test/utils/Print.hpp>

#include <srslib_framework/filter/FilterState.hpp>
#include <srslib_framework/filter/Command.hpp>
#include <srslib_framework/filter/Measurement.hpp>
#include <srslib_framework/filter/ukf/UnscentedKalmanFilter.hpp>
#include <srslib_framework/robotics/Pose.hpp>
#include <srslib_framework/robotics/Odometry.hpp>
#include <srslib_framework/robotics/Imu.hpp>

#include <srsnode_motion/PositionUkf.hpp>
#include <srsnode_motion/Robot.hpp>
#include <srsnode_motion/StatePe.hpp>
#include <srsnode_motion/CmdVelocity.hpp>

#include <srsnode_motion/tap/sensor_frame/odometry/OdometrySensor.hpp>
#include <srsnode_motion/tap/sensor_frame/imu/ImuSensor.hpp>

using namespace srs;

constexpr unsigned int UKF_STATE_SIZE = 5;
constexpr double DT = 1.0 / 100.0;

#include "data/UkfTestData_StraightOdometryImu.hpp"

TEST(Test_Motion, StraightOdometryImu)
{
    cv::Mat ROBOT_Q = cv::Mat::diag((cv::Mat_<double>(1, UKF_STATE_SIZE) <<
        pow(0.0007, 2.0), // [m^2]
        pow(0.0007, 2.0), // [m^2]
        pow(0.0017, 2.0), // [rad^2]
        0.0, // [m^2/s^2]
        0.0 // [m^2/s^2]
    ));

    cv::Mat ODOMETRY_R = cv::Mat::diag((cv::Mat_<double>(1, UKF_STATE_SIZE) <<
        0.0, // [m^2]
        0.0, // [m^2]
        0.0, // [rad^2]
        pow(0.000005, 2.0), // [m^2/s^2]
        0.0 // [m^2/s^2]
    ));

    cv::Mat IMU_R = cv::Mat::diag((cv::Mat_<double>(1, UKF_STATE_SIZE) <<
        0.0, // [m^2]
        0.0, // [m^2]
        0.0, // [rad^2]
        0.0, // [m^2/s^2]
        pow(0.00001, 2.0) // [m^2/s^2]
    ));

    // Create standard robot process model and initialize the process noise matrix
    Robot<> robot(ROBOT_Q + ODOMETRY_R);
    PositionUkf ukf(robot);

    // Add the IMU sensor
    ImuSensor<> imu(IMU_R);
    ukf.addSensor(&imu);

    // Create the sequence of odometry readings
    vector<const Odometry<>*> odometryReadings = {
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

    // Create the sequence of IMU readings
    vector<const Imu<>*> imuReadings = {
        &IMU_STEP_00,
        &IMU_STEP_01,
        &IMU_STEP_02,
        &IMU_STEP_03,
        &IMU_STEP_04,
        &IMU_STEP_05,
        &IMU_STEP_06,
        &IMU_STEP_07,
        &IMU_STEP_08,
        &IMU_STEP_09,
        &IMU_STEP_10,
        &IMU_STEP_11,
        &IMU_STEP_12,
        &IMU_STEP_13
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
    Pose<double> pose0 = Pose<>(1, 1, 0);
    StatePe<> stateT0(pose0);

    cv::Mat P0 = ROBOT_Q + ODOMETRY_R + IMU_R;
    ukf.reset(stateT0.getVectorForm(), P0);

    for (unsigned int t = 0; t < odometryReadings.size(); ++t)
    {
        // Push the simulated measurement
        auto odometryData = *odometryReadings.at(t);
        auto imuData = *imuReadings.at(t);

        // Run the step of the UKF
        imu.set(imuData);
        ukf.run(DT, &odometryData);

        ASSERT_TRUE(test::Compare::similar<>(ukf.getX(), correctStates[t], 1e-2)) <<
            " State vector at time-step " << t;
    }
}
