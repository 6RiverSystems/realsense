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

#include <srslib_framework/filter/Sensor.hpp>
#include <srslib_framework/robotics/Pose.hpp>
#include <srslib_framework/robotics/Velocity.hpp>
#include <srslib_framework/ros/RosTap.hpp>
#include <srslib_framework/ros/tap/RosTapBrainStem.hpp>
#include <srslib_framework/ros/tap/RosTapInitialPose.hpp>

#include <srsnode_motion/Configuration.hpp>
#include <srsnode_motion/Robot.hpp>
#include <srsnode_motion/StatePe.hpp>

#include <srslib_test/utils/Print.hpp>

#include <srsnode_motion/tap/odometry/RosTapOdometry.hpp>
#include <srsnode_motion/PositionUkf.hpp>

namespace srs {

class PositionEstimator
{
public:
    PositionEstimator(double refreshRate) :
        refreshRate_(refreshRate),
        ukf_(robot_),
        previousReadingTime_(-1.0)
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

        // Calculate the elapsed time between odometry readings
        double currentTime = odometry.velocity.arrivalTime;
        double dT = currentTime - previousReadingTime_;
        if (previousReadingTime_ < 0 || dT < 0)
        {
            dT = 0.0;
        }
        previousReadingTime_ = currentTime;

        ROS_INFO_STREAM_NAMED("PositionEstimator", "dT: " << dT);
        ukf_.run(dT, odometry);
    }

private:
    double previousReadingTime_;

    double refreshRate_;
    Robot<> robot_;

    PositionUkf ukf_;
};

} // namespace srs

#endif  // POSITIONESTIMATOR_HPP_
