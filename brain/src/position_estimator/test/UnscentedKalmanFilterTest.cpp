/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#include <gtest/gtest.h>

#include <vector>
using namespace std;

//#include <geometry_msgs/Twist.h>

#include <framework/Pose.hpp>

#include <filter/FilterState.hpp>
#include <filter/Command.hpp>
#include <filter/ukf/UnscentedKalmanFilter.hpp>

#include <sensor/odometry/OdometrySensor.hpp>

#include <Robot.hpp>

const unsigned int UKF_STATE_SIZE = 5;
const unsigned int UKF_COMMAND_SIZE = 2;

// Declaration and initialization test
TEST(UnscentedKalmanFilter, CreateFilter)
{
    // Create standard robot process model
    srs::Robot robot;

    srs::UnscentedKalmanFilter<UKF_STATE_SIZE> ukf(1.0, 0.0, robot);
}

TEST(UnscentedKalmanFilter, Run11Steps)
{
    // Create standard robot process model
    srs::Robot robot;

    srs::UnscentedKalmanFilter<UKF_STATE_SIZE> ukf(1.0, 0.0, robot);

    // Create a sequence of commands
    srs::Command<> COMMAND_1M_S(1, 0);

    vector<srs::Command<>*> commands = {
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

    // Prepare a sequence of odometry readings
    srs::Odometry ODOMETRY_0(0, 0, 0);
    srs::Odometry ODOMETRY_1M_S(1, 1, 0);

    vector<vector<srs::Measurement>> measurements = {
        vector<srs::Measurement>() = {ODOMETRY_1M_S},
        vector<srs::Measurement>() = {ODOMETRY_1M_S},
        vector<srs::Measurement>() = {ODOMETRY_1M_S},
        vector<srs::Measurement>() = {ODOMETRY_1M_S},
        vector<srs::Measurement>() = {ODOMETRY_1M_S},
        vector<srs::Measurement>() = {ODOMETRY_1M_S},
        vector<srs::Measurement>() = {ODOMETRY_1M_S},
        vector<srs::Measurement>() = {ODOMETRY_1M_S},
        vector<srs::Measurement>() = {ODOMETRY_1M_S},
        vector<srs::Measurement>() = {ODOMETRY_1M_S},
        vector<srs::Measurement>() = {ODOMETRY_0}
    };

    // Prepare the initial state
    srs::Pose<double> pose0;
    pose0.setThetaDegrees(90.0);
    cout << pose0.toString() << endl;

    srs::FilterState<UKF_STATE_SIZE> stateT0(pose0);

    cv::Mat covarianceT0 = cv::Mat::eye(UKF_STATE_SIZE, UKF_STATE_SIZE, CV_64F);
    covarianceT0 = covarianceT0 * 0.0001;

    ukf.reset(stateT0, covarianceT0);

    for (unsigned int t = 0; t < measurements.size(); ++t)
    {
        ukf.run(commands.at(t), measurements.at(t));
        //ukf.run(commands.at(t), measurements.at(t));
    }
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
