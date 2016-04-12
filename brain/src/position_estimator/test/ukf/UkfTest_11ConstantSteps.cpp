/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#include <gtest/gtest.h>

#include <vector>
using namespace std;

//#include <geometry_msgs/Twist.h>
#include <framework/Compare.hpp>

#include <framework/Utils.hpp>
#include <framework/Pose.hpp>

#include <filter/FilterState.hpp>
#include <filter/Command.hpp>
#include <filter/Measurement.hpp>
#include <filter/ukf/UnscentedKalmanFilter.hpp>

#include <sensor/odometry/Odometry.hpp>
#include <sensor/odometry/Odometer.hpp>

#include <RobotProfile.hpp>
#include <Robot.hpp>

using namespace srs;

constexpr unsigned int UKF_STATE_SIZE = 5;
constexpr unsigned int UKF_COMMAND_SIZE = 2;

constexpr double ALPHA = 1.0;
constexpr double BETA = 0.0;
constexpr double DT = 1.0;

#include "data/UkfTestData_11ConstantSteps.hpp"

TEST(UnscentedKalmanFilter, Run11ConstantSteps)
{
    // Create standard robot process model
    Robot<> robot;
    Odometer<> odometer(RobotProfile<>::SIZE_WHEEL_DISTANCE);
    UnscentedKalmanFilter<UKF_STATE_SIZE> ukf(ALPHA, BETA, robot, DT);

    // Create a sequence of commands
    Command<> COMMAND_1M_S = Command<>(1, 0);

    // Prepare a sequence of odometry readings
    Odometry<> ODOMETRY_0 = Odometry<>(&odometer, 0, 0, 0);
    Odometry<> ODOMETRY_1M_S = Odometry<>(&odometer, 2222, 1, 1);

    vector<Command<>*> commands = {
        &COMMAND_1M_S,
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

    vector<vector<Measurement<>*>> measurements = {
        vector<Measurement<>*>() = {&ODOMETRY_1M_S},
        vector<Measurement<>*>() = {&ODOMETRY_1M_S},
        vector<Measurement<>*>() = {&ODOMETRY_1M_S},
        vector<Measurement<>*>() = {&ODOMETRY_1M_S},
        vector<Measurement<>*>() = {&ODOMETRY_1M_S},
        vector<Measurement<>*>() = {&ODOMETRY_1M_S},
        vector<Measurement<>*>() = {&ODOMETRY_1M_S},
        vector<Measurement<>*>() = {&ODOMETRY_1M_S},
        vector<Measurement<>*>() = {&ODOMETRY_1M_S},
        vector<Measurement<>*>() = {&ODOMETRY_1M_S},
        vector<Measurement<>*>() = {&ODOMETRY_0}
    };

    vector<cv::Mat> correctCovariance = {
        COV_STEP_01,
        COV_STEP_02,
        COV_STEP_03,
        COV_STEP_04,
        COV_STEP_05,
        COV_STEP_06,
        COV_STEP_07,
        COV_STEP_08,
        COV_STEP_09,
        COV_STEP_10,
        COV_STEP_11
    };

    // Prepare the initial state
    Pose<double> pose0;
    pose0.setThetaDegrees(90.0);

    FilterState<> stateT0(pose0);

    cv::Mat covarianceT0 = cv::Mat::eye(UKF_STATE_SIZE, UKF_STATE_SIZE, CV_64F);
    covarianceT0 = covarianceT0 * 0.0001;

    ukf.reset(stateT0, covarianceT0);

    for (unsigned int t = 0; t < measurements.size(); ++t)
    {
        ukf.run(commands.at(t), measurements.at(t));

        cv::Mat covariance = ukf.getCovariance();
        ASSERT_TRUE(test::Compare::similar<>(covariance, correctCovariance.at(t), 0.1)) <<
            "Covariance at time-step " << t << " is not as expected.";
    }
}
