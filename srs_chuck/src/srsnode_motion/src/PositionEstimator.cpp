#include <srsnode_motion/PositionEstimator.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
void PositionEstimator::run(Odometry<>* odometry)
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

        // Update the accumulated odometry
        // using the physical model of the robot
        updateAccumulatedOdometry(dT, *odometry);
    }

    ukf_.run(dT, odometry);
}


////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

////////////////////////////////////////////////////////////////////////////////////////////////////
void PositionEstimator::updateAccumulatedOdometry(double dT, Odometry<> odometry)
{
    StatePe<> zeroState;
    zeroState.velocity = odometry.velocity;

    StatePe<> newState;
    robot_.kinematics(zeroState, dT, newState);

    accumulatedOdometry_ = PoseMath::add<double>(accumulatedOdometry_, newState.getPose());
}

} // namespace srs
