#include <srsnode_position_estimator/PositionEstimator.hpp>

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
PositionEstimator::PositionEstimator() :
    ukf_(ALPHA, BETA, robot_),
    rosNodeHandle_(),
    commandUpdated_(false)
{
    currentCovariance_ = robot_.getNoiseMatrix();
    currentState_ = StatePe<>(3.0, 2.0, 0.0);

    ukf_.addSensor(tapOdometer_.getSensor());
    ukf_.reset(currentState_.getVectorForm(), currentCovariance_);

    rosPubOdom_ = rosNodeHandle_.advertise<nav_msgs::Odometry>("/odom", 50);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void PositionEstimator::run()
{
    tapBrainStemStatus_.connectTap();
    tapCmdVel_.connectTap();
    tapOdometer_.connectTap();

    ros::Rate refreshRate(REFRESH_RATE_HZ);
    while (ros::ok())
    {
        ros::spinOnce();
        currentTime_ = ros::Time::now();

        scanTapsForData();
        stepUkf();
        publishInformation();

        refreshRate.sleep();
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

////////////////////////////////////////////////////////////////////////////////////////////////////
void PositionEstimator::disconnectAllTaps()
{
    tapBrainStemStatus_.disconnectTap();
    tapCmdVel_.disconnectTap();
    tapOdometer_.disconnectTap();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void PositionEstimator::publishInformation()
{
    geometry_msgs::Quaternion orientation = tf::createQuaternionMsgFromYaw(currentState_.theta);

    // Publish the TF of the pose
    geometry_msgs::TransformStamped messagePoseTf;
    messagePoseTf.header.frame_id = "odom";
    messagePoseTf.child_frame_id = "base_footprint";

    messagePoseTf.header.stamp = currentTime_;
    messagePoseTf.transform.translation.x = currentState_.x;
    messagePoseTf.transform.translation.y = currentState_.y;
    messagePoseTf.transform.translation.z = 0.0;
    messagePoseTf.transform.rotation = orientation;

    rosTfBroadcaster_.sendTransform(messagePoseTf);

    // Publish the required odometry for the planners
    nav_msgs::Odometry messagePose;
    messagePose.header.stamp = currentTime_;
    messagePose.header.frame_id = "odom";
    messagePose.child_frame_id = "base_footprint";

    // Position
    messagePose.pose.pose.position.x = currentState_.x;
    messagePose.pose.pose.position.y = currentState_.y;
    messagePose.pose.pose.position.z = 0.0;
    messagePose.pose.pose.orientation = orientation;

    // Velocity
    messagePose.twist.twist.linear.x = currentState_.v;
    messagePose.twist.twist.linear.y = 0.0;
    messagePose.twist.twist.linear.z = 0.0;
    messagePose.twist.twist.angular.x = 0.0;
    messagePose.twist.twist.angular.y = 0.0;
    messagePose.twist.twist.angular.z = currentState_.omega;

    // Publish the Odometry
    rosPubOdom_.publish(messagePose);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void PositionEstimator::scanTapsForData()
{
    commandUpdated_ = tapCmdVel_.newDataAvailable();
    currentCommand_ = CmdVelocity<>(tapCmdVel_.getCurrentData());

    // If the brain stem is disconnected, simulate odometry
    // feeding the odometer the commanded velocity
    if (!tapBrainStemStatus_.isBrainStemConnected())
    {
        tapOdometer_.set(currentTime_.nsec,
            currentCommand_.velocity.linear,
            currentCommand_.velocity.angular);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void PositionEstimator::stepUkf()
{
    // Calculate how many seconds have passed from the previous update
    previousTimeNs_ = currentTimeNs_;
    currentTimeNs_ = static_cast<double>(currentTime_.nsec) * 1.0e-9;

    previousTimeS_ = currentTimeS_;
    currentTimeS_ = static_cast<double>(currentTime_.sec);

    double dT = currentTimeNs_ + (currentTimeS_ - previousTimeS_) - previousTimeNs_;

    // Advance the state of the UKF
    ukf_.run(dT, commandUpdated_ ? &currentCommand_ : nullptr);

    // Store the new state and covariance for publication
    currentState_ = StatePe<>(ukf_.getState());
    currentCovariance_ = ukf_.getCovariance();
}

} // namespace srs
