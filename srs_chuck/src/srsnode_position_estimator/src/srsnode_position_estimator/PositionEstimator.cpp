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
    currentState_ = PEState<>(3.0, 2.0, 0.0);

    ukf_.reset(currentState_.getVectorForm(), currentCovariance_);

    rosPubOdom_ = rosNodeHandle_.advertise<nav_msgs::Odometry>("/odom", 50);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void PositionEstimator::run()
{
    brainStemStatusTap_.connectTap();
    velCmdTap_.connectTap();
    odometerTap_.connectTap();

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
    brainStemStatusTap_.disconnectTap();
    odometerTap_.disconnectTap();
    velCmdTap_.disconnectTap();
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
    commandUpdated_ = velCmdTap_.newDataAvailable();
    currentCommand_ = velCmdTap_.getCurrentData();

    if (!brainStemStatusTap_.isBrainStemConnected())
    {
        odometerTap_.set(currentTime_.nsec, currentCommand_.v, currentCommand_.omega);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void PositionEstimator::stepUkf()
{
    ukf_.run(currentTime_.sec, commandUpdated_ ? &currentCommand_ : nullptr);
    currentState_ = PEState<>(ukf_.getState());
    currentCovariance_ = ukf_.getCovariance();
}

} // namespace srs
