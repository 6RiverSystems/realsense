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

#include <srsnode_motion/Configuration.hpp>
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

    void run(Odometry<>* odometry)
    {
        double dT = dT_;
        if (odometry)
        {
            ROS_DEBUG_STREAM_THROTTLE_NAMED(1.0, "PositionEstimator",
                "Position Estimator Odometry: " << *odometry);

            // Calculate the elapsed time between odometry readings
            double currentTime = odometry->velocity.arrivalTime;
            dT = currentTime - previousReadingTime_;
            if (previousReadingTime_ < 0 || dT < 0)
            {
                dT = dT_;
            }
            previousReadingTime_ = currentTime;

            ROS_DEBUG_STREAM_THROTTLE_NAMED(1.0,
                "PositionEstimator", "Calculated dT: " << dT);
        }

        ukf_.run(dT, odometry);
    }

private:
    double dT_;

    double previousReadingTime_;

    Robot<> robot_;

    PositionUkf ukf_;
};

} // namespace srs

#endif  // POSITIONESTIMATOR_HPP_
