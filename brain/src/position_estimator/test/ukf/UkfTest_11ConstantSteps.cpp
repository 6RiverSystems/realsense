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
#include <PEState.hpp>
#include <VelCmd.hpp>

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
    Odometer<UKF_STATE_SIZE> odometer(RobotProfile<>::SIZE_WHEEL_DISTANCE);

    UnscentedKalmanFilter<UKF_STATE_SIZE> ukf(ALPHA, BETA, robot, DT);
    ukf.addSensor(&odometer);

    // Create a sequence of commands
    VelCmd<> COMMAND_1M_S = VelCmd<>(1, 0);

    // Prepare a sequence of odometry readings
    Odometry<> ODOMETRY_0 = Odometry<>(0, 0, 0);
    Odometry<> ODOMETRY_1M_S = Odometry<>(2222, 1, 1);

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

    vector<vector<Measurement*>> measurements = {
        vector<Measurement*>() = {&ODOMETRY_1M_S},
        vector<Measurement*>() = {&ODOMETRY_1M_S},
        vector<Measurement*>() = {&ODOMETRY_1M_S},
        vector<Measurement*>() = {&ODOMETRY_1M_S},
        vector<Measurement*>() = {&ODOMETRY_1M_S},
        vector<Measurement*>() = {&ODOMETRY_1M_S},
        vector<Measurement*>() = {&ODOMETRY_1M_S},
        vector<Measurement*>() = {&ODOMETRY_1M_S},
        vector<Measurement*>() = {&ODOMETRY_1M_S},
        vector<Measurement*>() = {&ODOMETRY_1M_S},
        vector<Measurement*>() = {&ODOMETRY_0}
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

    // Prepare the initial state
    Pose<double> pose0;
    pose0.setThetaDegrees(90.0);

    PEState<> stateT0(pose0);

    cv::Mat covarianceT0 = cv::Mat::eye(UKF_STATE_SIZE, UKF_STATE_SIZE, CV_64F);
    covarianceT0 = covarianceT0 * 0.0001;

    ukf.reset(stateT0.getStateVector(), covarianceT0);

    for (unsigned int t = 0; t < measurements.size(); ++t)
    {
        ukf.run(commands.at(t));

        ASSERT_TRUE(test::Compare::similar<>(ukf.getCovariance(), correctCovariances[t], 2e-2)) <<
            " Covariance matrix at time-step " << t;
        ASSERT_TRUE(test::Compare::similar<>(ukf.getState(), correctStates[t], 1e-1)) <<
            " State vector at time-step " << t;
    }
}
