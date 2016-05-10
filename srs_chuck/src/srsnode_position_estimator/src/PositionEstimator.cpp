#include <srsnode_position_estimator/PositionEstimator.hpp>

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>

#include <srslib_framework/math/Time.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
PositionEstimator::PositionEstimator(string nodeName) :
    commandUpdated_(false),
    rosNodeHandle_(nodeName),
    tapBrainStemStatus_(nodeName),
    tapCmdVel_(nodeName),
    tapOdometry_(nodeName),
    tapInitialPose_(nodeName),
    ukf_(ALPHA, BETA, robot_)
{
    currentCovariance_ = robot_.getNoiseMatrix();
    currentState_ = StatePe<>(2.0, 1.5, 0.0);

    ukf_.addSensor(tapOdometry_.getSensor());
    // ukf_.addSensor(tapAps_.getSensor());
    ukf_.reset(currentState_.getVectorForm(), currentCovariance_);

    rosPubOdom_ = rosNodeHandle_.advertise<nav_msgs::Odometry>("/odom", 50);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void PositionEstimator::run()
{
    tapBrainStemStatus_.connectTap();
    tapCmdVel_.connectTap();
    tapOdometry_.connectTap();
    tapInitialPose_.connectTap();

    ros::Rate refreshRate(REFRESH_RATE_HZ);
    while (ros::ok())
    {
        ros::spinOnce();

        previousTime_ = currentTime_;
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
    tapOdometry_.disconnectTap();
    tapInitialPose_.disconnectTap();
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
        tapOdometry_.set(Time::time2number(currentTime_),
            currentCommand_.velocity.linear,
            currentCommand_.velocity.angular);
    }

    if (tapInitialPose_.newDataAvailable())
    {
        currentCovariance_ = robot_.getNoiseMatrix();
        Pose<> ZERO(2.0, 1.5, 0.0);
        currentState_ = StatePe<>(ZERO/*tapInitialPose_.getInitialPose()*/);

        ukf_.reset(currentState_.getVectorForm(), currentCovariance_);
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

    double dT1 = currentTimeNs_ + (currentTimeS_ - previousTimeS_) - previousTimeNs_;

    double cT = Time::time2number(currentTime_);
    double pT = Time::time2number(previousTime_);
    double dT2 =  cT - pT;

    if (abs(dT1 - dT2) > 0.1)
    {
        ROS_ERROR_STREAM("--------------------------------");
        ROS_ERROR_STREAM("dT1: " << dT1);
        ROS_ERROR_STREAM("dT2: " << dT2);
        ROS_ERROR_STREAM("pTns: " << previousTimeNs_);
        ROS_ERROR_STREAM("cTns: " << currentTimeNs_);
        ROS_ERROR_STREAM("pTs: " << previousTimeS_);
        ROS_ERROR_STREAM("cTs: " << currentTimeS_);
        ROS_ERROR_STREAM("cT: " << cT);
        ROS_ERROR_STREAM("pT: " << pT);
        ROS_ERROR_STREAM("--------------------------------");
    }

    // Advance the state of the UKF
    ukf_.run(dT2, commandUpdated_ ? &currentCommand_ : nullptr);

    // Store the new state and covariance for publication
    currentState_ = StatePe<>(ukf_.getState());
    currentCovariance_ = ukf_.getCovariance();
}

} // namespace srs
