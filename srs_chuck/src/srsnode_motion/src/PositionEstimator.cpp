#include <srsnode_motion/PositionEstimator.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
PositionEstimator::PositionEstimator(double dT) :
    dTDefault_(dT),
    initialized_(false),
    naiveSensorFusion_(true),
    ukf_(robot_),
    previousNodeReadingTime_(-1.0),
    previousOdometryTime_(-1.0),
    dTNode_(0.0),
    dTOdometry_(0.0),
    p0_(10.0)
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////
PositionEstimator::~PositionEstimator()
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void PositionEstimator::reset(Pose<> initialPose)
{
    if (initialPose.isValid())
    {
        initialized_ = true;

        StatePe<> currentState = StatePe<>(initialPose);

        cv::Mat P0 = (cv::Mat_<double>(1, STATIC_UKF_STATE_VECTOR_SIZE) <<
            pow(p0_, 2.0),
            pow(p0_, 2.0),
            pow(p0_, 2.0),
            pow(p0_, 2.0),
            pow(p0_, 2.0)
        );

        ukf_.reset(currentState.getVectorForm(), cv::Mat::diag(P0));
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void PositionEstimator::resetAccumulatedOdometry(Pose<>* initialPose)
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

////////////////////////////////////////////////////////////////////////////////////////////////////
void PositionEstimator::run(Odometry<>* odometry, Imu<>* imu, Pose<>* aps)
{
    // Calculate the elapsed time between node execution
    double currentTime = ros::Time::now().toSec();
    dTNode_ = currentTime - previousNodeReadingTime_;
    if (previousNodeReadingTime_ < 0 || dTNode_ < 0)
    {
        ROS_DEBUG_STREAM_THROTTLE_NAMED(1.0,
            "position_estimator", "Synchronizing dT to " << dTDefault_);
        dTNode_ = dTDefault_;
    }
    previousNodeReadingTime_ = currentTime;

    ROS_DEBUG_STREAM_THROTTLE_NAMED(1.0,
        "position_estimator", "Calculated node dT: " << dTNode_);

    if (odometry)
    {
        ROS_DEBUG_STREAM_THROTTLE_NAMED(1.0, "position_estimator",
            "Position Estimator Odometry: " << *odometry);

        // Calculate the elapsed time between node execution
        dTOdometry_ = odometry->velocity.arrivalTime - previousOdometryTime_;
        if (previousOdometryTime_ < 0 || dTOdometry_ < 0)
        {
            ROS_DEBUG_STREAM_THROTTLE_NAMED(1.0,
                "position_estimator", "Synchronizing dT to " << dTDefault_);
            dTOdometry_ = dTDefault_;
        }
        previousOdometryTime_ = odometry->velocity.arrivalTime;

        ROS_DEBUG_STREAM_THROTTLE_NAMED(1.0,
            "position_estimator", "Calculated odometry dT: " << dTOdometry_);

        // Update the accumulated odometry
        // using the physical model of the robot
        updateAccumulatedOdometry(dTOdometry_, *odometry);
    }

    // Do not run the UKF if the position estimator was not
    // correctly initialized with a position (manually provided
    // or by the APS
    if (initialized_)
    {
        if (naiveSensorFusion_)
        {
            runNaiveSensorFusion(dTOdometry_, odometry, imu, aps);
        }
        else
        {
            ukf_.run(dTOdometry_/* ###FS dTNode_*/, odometry);
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

void PositionEstimator::runNaiveSensorFusion(double dT, Odometry<>* odometry, Imu<>* imu, Pose<>* aps)
{
    ROS_WARN_STREAM("Valid pose naive");
    StatePe<> currentState = StatePe<>(ukf_.getX());

    if (aps)
    {
        currentState.pose = *aps;
        ROS_WARN_STREAM("New pose: " << *aps);

        ukf_.reset(currentState.getVectorForm());
    }
    else
    {
        if (imu && odometry)
        {
            currentState.pose.theta = imu->yaw;
            ROS_WARN_STREAM("New theta: " << imu->yaw);

            double angular = (imu->yaw - currentState.pose.theta) / dT;
            currentState.velocity = Velocity<>(odometry->velocity.linear, angular);

            ROS_WARN_STREAM("New velocity: " << currentState.velocity);

            StatePe<> newState;
            robot_.kinematics(currentState, dT, newState);

            ROS_WARN_STREAM("New state: " << newState);

            ukf_.reset(newState.getVectorForm());
        }
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
