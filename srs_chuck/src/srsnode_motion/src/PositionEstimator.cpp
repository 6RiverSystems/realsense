#include <srsnode_motion/PositionEstimator.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
PositionEstimator::PositionEstimator(double dT) :
    dT_(dT),
    initialized_(false),
    naiveSensorFusion_(true),
    ukf_(robot_),
    previousReadingTime_(-1.0)
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////
PositionEstimator::~PositionEstimator()
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void PositionEstimator::run(Odometry<>* odometry, Imu<>* imu, Pose<>* aps)
{
    double currentTime = ros::Time::now().toSec();

    // Calculate the elapsed time between odometry readings
    double dT = currentTime - previousReadingTime_;
    if (previousReadingTime_ < 0 || dT < 0)
    {
        ROS_DEBUG_STREAM_THROTTLE_NAMED(1.0,
            "position_estimator", "Synchronizing dT to " << dT_);
        dT = dT_;
    }
    previousReadingTime_ = currentTime;

    ROS_DEBUG_STREAM_THROTTLE_NAMED(1.0,
        "position_estimator", "Calculated dT: " << dT);

    if (odometry)
    {
        ROS_DEBUG_STREAM_THROTTLE_NAMED(1.0, "position_estimator",
            "Position Estimator Odometry: " << *odometry);

        // Update the accumulated odometry
        // using the physical model of the robot
        updateAccumulatedOdometry(dT_, *odometry);
    }

    if (naiveSensorFusion_)
    {
        runNaiveSensorFusion(dT, odometry, imu, aps);
    }
    else {
        // Do not run the UKF if the position estimator was not
        // correctly initialized with a position (manually provided
        // or by the APS
        if (initialized_)
        {
            ukf_.run(dT, odometry);
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

void PositionEstimator::runNaiveSensorFusion(double dT, Odometry<>* odometry, Imu<>* imu, Pose<>* aps)
{
    if (isPoseValid())
    {
        StatePe<> currentState = StatePe<>(ukf_.getX());

        if (aps)
        {
            currentState.pose = *aps;
        }
        else
        {
            if (imu)
            {
                currentState.pose.theta = imu->yaw;
            }
        }

        if (imu && odometry)
        {
            double angular = (imu->yaw - currentState.pose.theta) / dT;
            Velocity<> velocity = Velocity<>(odometry->velocity.linear, angular);

            currentState.velocity = velocity;
        }

        StatePe<> newState;
        robot_.kinematics(currentState, dT, newState);

        ukf_.reset(newState.getVectorForm(), robot_.getQ());
    }
}

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
