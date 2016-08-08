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

#include <srsnode_motion/MotionConfig.h>
using namespace srsnode_motion;

#include <srslib_framework/filter/Sensor.hpp>
#include <srslib_framework/robotics/Pose.hpp>
#include <srslib_framework/robotics/Velocity.hpp>
#include <srslib_framework/robotics/Imu.hpp>

#include <srsnode_motion/Configuration.hpp>
#include <srsnode_motion/FactoryRobotNoise.hpp>
#include <srsnode_motion/Robot.hpp>
#include <srsnode_motion/StatePe.hpp>

#include <srslib_test/utils/Print.hpp>

#include <srsnode_motion/PositionUkf.hpp>

namespace srs {

class PositionEstimator
{
public:
    PositionEstimator(double dT);
    ~PositionEstimator();

    void addSensor(Sensor<>* sensor)
    {
        ukf_.addSensor(sensor);
    }

    void enableNaive(bool newState)
    {
        naiveSensorFusion_ = newState;
    }

    Pose<> getAccumulatedOdometry()
    {
        return accumulatedOdometry_;
    }

    Pose<> getPose()
    {
        if (initialized_)
        {
            StatePe<> currentState = StatePe<>(ukf_.getX());
            return currentState.getPose();
        }

        return Pose<>::INVALID;
    }

    Velocity<> getVelocity()
    {
        if (initialized_)
        {
            StatePe<> currentState = StatePe<>(ukf_.getX());
            return currentState.getVelocity();
        }

        return Velocity<>::INVALID;
    }

    bool isPoseValid()
    {
        Pose<> currentPose = getPose();
        return currentPose.isValid();
    }

    void reset(Pose<> initialPose)
    {
        if (initialPose.isValid())
        {
            initialized_ = true;

            StatePe<> currentState = StatePe<>(initialPose);
            cv::Mat currentCovariance = robot_.getQ();

            ukf_.reset(currentState.getVectorForm(), currentCovariance);
        }
    }

    void resetAccumulatedOdometry(Pose<>* initialPose)
    {
        if (initialPose)
        {
            accumulatedOdometry_ = *initialPose;
        }
        else
        {
            accumulatedOdometry_ = Pose<>::ZERO;
        }
    }

    void run(Odometry<>* odometry, Imu<>* imu, Pose<>* aps);

    void setRobotQ(cv::Mat Q)
    {
        robot_.setQ(Q);
    }

private:
    void runNaiveSensorFusion(double dT, Odometry<>* odometry, Imu<>* imu, Pose<>* aps);

    void updateAccumulatedOdometry(double dT, Odometry<> odometry);

    Pose<> accumulatedOdometry_;

    double dTDefault_;
    double dTOdometry_;
    double dTNode_;

    bool initialized_;

    bool naiveSensorFusion_;

    double previousNodeReadingTime_;
    double previousOdometryTime_;

    Robot<> robot_;

    PositionUkf ukf_;
};

} // namespace srs

#endif  // POSITIONESTIMATOR_HPP_
