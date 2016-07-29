#include <srsnode_motion/PositionEstimator.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
PositionEstimator::PositionEstimator(double dT) :
    dT_(dT),
    initialized_(false),
    ukf_(robot_),
    previousReadingTime_(-1.0)
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////
PositionEstimator::~PositionEstimator()
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void PositionEstimator::run(Odometry<>* odometry, Imu<> imu)
{
    double dT = dT_;

    imu_ = imu;

    if (odometry)
    {
        ROS_DEBUG_STREAM_THROTTLE_NAMED(1.0, "position_estimator",
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
            "position_estimator", "Calculated dT: " << dT);

        // Update the accumulated odometry
        // using the physical model of the robot
        updateAccumulatedOdometry(dT, *odometry);
    }

    // Do not run the UKF if the position estimator was not
    // correctly initialized with a position (manually provided
    // or by the APS
    if (initialized_)
    {
        //ukf_.setTheta(imu_.yaw);
        ukf_.run(dT, odometry);
    }
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
