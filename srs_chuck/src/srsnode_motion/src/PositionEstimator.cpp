#include <srsnode_motion/PositionEstimator.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
PositionEstimator::PositionEstimator(double dT) :
    dTDefault_(dT),
    initialized_(false),
    naiveSensorFusion_(true),
    neverSeenAps_(true),
    ukf_(robot_),
    previousNodeReadingTime_(-1.0),
    previousOdometryTime_(-1.0),
    dTNode_(0.0),
    dTOdometry_(0.0),
    p0_(10.0),
    sumDeltaTheta_(0.0),
    previousImu_(Imu<>::INVALID),
    apsTimeout_(0),
    apsCounter_(0),
    correctedApsTheta_(0.0)
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

        apsTimeout_ = 0;
        apsCounter_ = 0;
        neverSeenAps_ = true;
        previousImu_ = Imu<>::INVALID;
        sumDeltaTheta_ = 0.0;
        correctedApsTheta_ = 0.0;
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
    StatePe<> currentState = StatePe<>(ukf_.getX());

    double imuDelta = 0.0;

    if (imu)
    {
        if (previousImu_.isValid())
        {
            imuDelta = (imu->yaw - previousImu_.yaw);
            sumDeltaTheta_ += imuDelta;
        }
        else
        {
            sumDeltaTheta_ = 0.0;
        }

        previousImu_ = *imu;

        apsTimeout_++;

        // ugly trick to reset the stargazer if we don't see a
        // reading for 100 IMU readings
        if (apsTimeout_ > 100)
        {
            neverSeenAps_ = true;
            apsCounter_ = 0;
            apsTimeout_ = 0;
        }
    }

    if (aps && previousImu_.isValid())
    {
        apsCounter_++;
        apsTimeout_ = 0;

        double historicAngle;
        if (neverSeenAps_)
        {
            historicAngle = aps->theta;
            neverSeenAps_ = false;
        }
        else
        {
            historicAngle = correctedApsTheta_ + sumDeltaTheta_;
        }
        sumDeltaTheta_ = 0.0;

        double currentSine = sin(aps->theta);
        double currentCosine = cos(aps->theta);
        double historicSine = sin(historicAngle);
        double historicCosine = cos(historicAngle);

        double averageSine = (historicSine * (apsCounter_ - 1) + currentSine) / apsCounter_;
        double averageCosine = (historicCosine * (apsCounter_ - 1) + currentCosine) / apsCounter_;

        correctedApsTheta_ = atan2(averageSine, averageCosine);

        currentState.pose = *aps;
        currentState.pose.theta = correctedApsTheta_;

        ukf_.reset(currentState.getVectorForm());
    }
    else if (imu && odometry)
    {
        double angular = imuDelta / dT;
        currentState.velocity = Velocity<>(odometry->velocity.linear, angular);

        StatePe<> newState;
        robot_.kinematics(currentState, dT, newState);

        ukf_.reset(newState.getVectorForm());
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
