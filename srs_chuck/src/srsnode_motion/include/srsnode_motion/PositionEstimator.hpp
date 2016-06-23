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
    PositionEstimator(double dT) :
        dT_(dT),
        ukf_(robot_),
        previousReadingTime_(-1.0)
    {}

    ~PositionEstimator()
    {}

    void addSensor(Sensor<>* sensor)
    {
        ukf_.addSensor(sensor);
    }

    Pose<> getAccumulatedOdometry()
    {
        return accumulatedOdometry_;
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

    void resetAccumulatedOdometry()
    {
        accumulatedOdometry_ = Pose<>();
    }

    void run(Odometry<>* odometry);

    void setConfiguration(MotionConfig& configuration)
    {
        cv::Mat newQ = FactoryRobotNoise::fromConfiguration(configuration);
        robot_.setQ(newQ);
    }

private:
    void updateAccumulatedOdometry(double dT, Odometry<> odometry);

    Pose<> accumulatedOdometry_;

    double dT_;

    double previousReadingTime_;

    Robot<> robot_;

    PositionUkf ukf_;
};

} // namespace srs

#endif  // POSITIONESTIMATOR_HPP_
