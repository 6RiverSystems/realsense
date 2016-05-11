#include <srsnode_motion/Motion.hpp>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#include <srslib_framework/math/Time.hpp>

#include <srslib_framework/graph/grid2d/Grid2d.hpp>
#include <srslib_framework/search/SearchPosition.hpp>
#include <srslib_framework/search/SolutionNode.hpp>
#include <srslib_framework/planning/pathplanning/Trajectory.hpp>
#include <srslib_framework/robotics/robot/Chuck.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
Motion::Motion(string nodeName) :
    rosNodeHandle_(nodeName),
    executionTime_(0.0),
    nextScheduledTime_(-1),
    commandUpdated_(false),
    tapBrainStemStatus_(nodeName),
    tapCmdVel_(nodeName),
    tapOdometry_(nodeName),
    tapInitialPose_(nodeName),
    tapPlan_(nodeName),
    ukf_(ALPHA, BETA, robot_)
{
    nextScheduled_ = trajectory_.end();

    currentCovariance_ = robot_.getNoiseMatrix();
    currentState_ = StatePe<>(2.0, 1.5, 0.0);

    ukf_.addSensor(tapOdometry_.getSensor());
    // ukf_.addSensor(tapAps_.getSensor());
    ukf_.reset(currentState_.getVectorForm(), currentCovariance_);

    pubCmdVel_ = rosNodeHandle_.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
    pubOdom_ = rosNodeHandle_.advertise<nav_msgs::Odometry>("/odom", 50);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Motion::run()
{
    tapBrainStemStatus_.connectTap();
    tapCmdVel_.connectTap();
    tapOdometry_.connectTap();
    tapInitialPose_.connectTap();
    tapPlan_.connectTap();

    ros::Rate refreshRate(REFRESH_RATE_HZ);
    while (ros::ok())
    {
        ros::spinOnce();

        previousTime_ = currentTime_;
        currentTime_ = ros::Time::now();

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

        scanTapsForData();
        stepUkf(dT2);
        stepMotionController(dT2);
        publishInformation();

        refreshRate.sleep();
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

////////////////////////////////////////////////////////////////////////////////////////////////////
void Motion::disconnectAllTaps()
{
    tapBrainStemStatus_.disconnectTap();
    tapCmdVel_.disconnectTap();
    tapOdometry_.disconnectTap();
    tapInitialPose_.disconnectTap();
    tapPlan_.disconnectTap();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Motion::publishInformation()
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
    pubOdom_.publish(messagePose);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Motion::scanTapsForData()
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
        currentState_ = StatePe<>(tapInitialPose_.getInitialPose());

        ukf_.reset(currentState_.getVectorForm(), currentCovariance_);
    }

    // If there is a new plan to execute
    if (tapPlan_.newDataAvailable())
    {
        trajectory_.clear();

        Chuck chuck;
        vector<SolutionNode<Grid2d>> solution = tapPlan_.getGoalPlan();

        Trajectory trajectoryConverter(solution, chuck, 1.0 / REFRESH_RATE_HZ);
        trajectoryConverter.solution2trajectory(trajectory_);

        for (auto milestone : trajectory_)
        {
            ROS_DEBUG_STREAM("Executing: " << milestone.first << " - " << milestone.second);
        }

        executionTime_ = 0.0;
        nextScheduled_ = trajectory_.begin();
        nextScheduledTime_ = nextScheduled_->first.arrivalTime;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Motion::stepUkf(double dT)
{
    // Advance the state of the UKF
    ukf_.run(dT, commandUpdated_ ? &currentCommand_ : nullptr);

    // Store the new state and covariance for publication
    currentState_ = StatePe<>(ukf_.getState());
    currentCovariance_ = ukf_.getCovariance();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Motion::stepMotionController(double dT)
{
    if (nextScheduledTime_ > -1)
    {
        ROS_DEBUG_STREAM("[" << executionTime_ << "] Waiting for " << nextScheduledTime_);
        executionTime_ += dT;

        if (executionTime_ >= nextScheduledTime_)
        {
            MilestoneType milestone = *nextScheduled_;

            ROS_DEBUG_STREAM("Executing: " << milestone.first << " - " << milestone.second);

            geometry_msgs::Twist message;

            message.linear.x = milestone.second.linear;
            message.linear.y = 0;
            message.linear.z = 0;
            message.angular.x = 0;
            message.angular.y = 0;
            message.angular.z = milestone.second.angular;

            pubCmdVel_.publish(message);

            nextScheduled_++;
            nextScheduledTime_ = nextScheduled_ != trajectory_.end() ?
                nextScheduled_->first.arrivalTime :
                -1;
        }
    }
}

} // namespace srs
