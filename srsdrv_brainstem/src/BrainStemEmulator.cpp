#include <BrainStemEmulator.hpp>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_datatypes.h>

#include <srslib_framework/chuck/ChuckTransforms.hpp>
#include <srslib_framework/math/PoseMath.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
BrainStemEmulator::BrainStemEmulator():
    currentPose_(Pose<>::ZERO),
    currentVelocity_(Velocity<>::ZERO)
{
    // Start the emulation tick timer
    tickTimer_ = ros::NodeHandle().createTimer(ros::Duration(1.0 / EMULATION_TICK_HZ),
        boost::bind(&BrainStemEmulator::tickEmulation, this, _1));

    brainstemTransform_.setIdentity();

    tapInitialPose_.attach(this);

    ROS_WARN("Brainstem emulator started");
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
BrainStemEmulator::~BrainStemEmulator()
{
    ROS_WARN("Brainstem emulator ended");
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void BrainStemEmulator::notified(Subscriber<geometry_msgs::PoseWithCovarianceStamped>* subject)
{
    TapInitialPose* tap = static_cast<TapInitialPose*>(subject);
    currentPose_ = tap->pop();

    ROS_DEBUG_STREAM("Notification received " << currentPose_);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void BrainStemEmulator::publishOdometry()
{
    nav_msgs::Odometry odometryMsg;

    odometryMsg.header.stamp = currentTime_;
    odometryMsg.header.frame_id = ChuckTransforms::ODOMETRY;
    odometryMsg.child_frame_id = ChuckTransforms::BASE_FOOTPRINT;

    // Position
    odometryMsg.pose.pose.position.x = currentPose_.x;
    odometryMsg.pose.pose.position.y = currentPose_.y;
    odometryMsg.pose.pose.position.z = 0.0;
    odometryMsg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(currentPose_.theta);

    // Velocity
    odometryMsg.twist.twist.linear.x = currentVelocity_.linear;
    odometryMsg.twist.twist.linear.y = 0.0;
    odometryMsg.twist.twist.linear.z = 0.0;
    odometryMsg.twist.twist.angular.x = 0.0;
    odometryMsg.twist.twist.angular.y = 0.0;
    odometryMsg.twist.twist.angular.z = currentVelocity_.angular;

    channelOdometryPose_.publish(odometryMsg);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void BrainStemEmulator::publishTransform()
{
    tf::Vector3 translation(currentPose_.x, currentPose_.y, 0.0);
    tf::Quaternion rotation = tf::createQuaternionFromYaw(currentPose_.theta);

    currentTransform_.setOrigin(translation);
    currentTransform_.setRotation(rotation);

    // calculate pose in GLOBAL frame
    // use StampedTransform because it can be converted to geometry_msgs::TransformStamped directly
    tf::StampedTransform currentTransform(brainstemTransform_ * currentTransform_,
        currentTime_, ChuckTransforms::ODOMETRY, ChuckTransforms::BASE_FOOTPRINT);

    // Publish the TF
    geometry_msgs::TransformStamped odom_trans;
    tf::transformStampedTFToMsg(currentTransform, odom_trans);

    tfBroadcaster_.sendTransform( odom_trans );

}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void BrainStemEmulator::tickEmulation(const ros::TimerEvent& event)
{
    currentTime_ = ros::Time::now();

    // First collect the most updated velocity
    currentVelocity_ = tapCmdVelocity_.pop();

    // Estimate the new pose based on the current pose, velocity, and simulated time
    currentPose_ = estimatePose(currentPose_, 1.0 / EMULATION_TICK_HZ, currentVelocity_);

    publishOdometry();
    publishTransform();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Pose<> BrainStemEmulator::estimatePose(const Pose<>& pose0, float dfTimeDelta, const Velocity<>& velocity)
{
    constexpr static double ANGULAR_VELOCITY_EPSILON = 0.000001; // [rad/s] (0.0573 [deg/s])

    float linearVelocity = velocity.linear;
    float angularVelocity = velocity.angular;

    Pose<> pose1;

    // Check for the special case in which omega is 0 (the robot is moving straight)
    if (abs(angularVelocity) > ANGULAR_VELOCITY_EPSILON)
    {
        double r = linearVelocity / angularVelocity;

        pose1.x = pose0.x + r * sin(pose0.theta + angularVelocity * dfTimeDelta) - r * sin(pose0.theta),
        pose1.y = pose0.y + r * cos(pose0.theta) - r * cos(pose0.theta + angularVelocity * dfTimeDelta),
        pose1.theta = AngleMath::normalizeRad<>(pose0.theta + angularVelocity * dfTimeDelta);
    }
    else
    {
        pose1 = PoseMath::translate<double>(pose0, linearVelocity * dfTimeDelta, 0.0);
    }

    return pose1;
}

}
