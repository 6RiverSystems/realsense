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
#include <srslib_framework/robotics/Odometry.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
Motion::Motion(string nodeName) :
    commandUpdated_(false),
    currentCommand_(),
    currentPose_(),
    rosNodeHandle_(nodeName),
    positionEstimator_(),
    motionController_(),
    tapPlan_(rosNodeHandle_),
    tapJoyAdapter_(rosNodeHandle_),
    tapBrainStemStatus_(rosNodeHandle_),
    tapOdometry_(rosNodeHandle_),
    tapInitialPose_(rosNodeHandle_),
    tapAps_(rosNodeHandle_),
    triggerShutdown_(rosNodeHandle_),
    triggerStop_(rosNodeHandle_, "Trigger: Stop", "trg/stop")
{
    pubCmdVel_ = rosNodeHandle_.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
    pubOdom_ = rosNodeHandle_.advertise<nav_msgs::Odometry>("/odom", 50);
    pubPose_ = rosNodeHandle_.advertise<geometry_msgs::PoseStamped>("pose", 10);

    positionEstimator_.addSensor(tapOdometry_.getSensor());
    positionEstimator_.addSensor(tapAps_.getSensor());
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Motion::reset()
{
    currentPose_ = tapInitialPose_.getPose();

    commandUpdated_ = false;
    currentCommand_ = Velocity<>();

    positionEstimator_.reset(currentPose_);
    motionController_.reset();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Motion::run()
{
    connectAllTaps();

    reset();

    ros::Rate refreshRate(REFRESH_RATE_HZ);
    while (ros::ok())
    {
        ros::spinOnce();

        previousTime_ = currentTime_;
        currentTime_ = ros::Time::now();

        double dT = Time::time2number(currentTime_) - Time::time2number(previousTime_);

        evaluateTriggers();
        scanTapsForData();

        Velocity<>* command = commandUpdated_ ? &currentCommand_ : nullptr;
        positionEstimator_.run(dT, command);
        motionController_.run(dT, positionEstimator_.getPose(), command);

        publishInformation();

        refreshRate.sleep();
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

////////////////////////////////////////////////////////////////////////////////////////////////////
void Motion::connectAllTaps()
{
    tapBrainStemStatus_.connectTap();
    tapOdometry_.connectTap();
    tapInitialPose_.connectTap();
    tapPlan_.connectTap();
    tapJoyAdapter_.connectTap();
    tapAps_.connectTap();

    triggerStop_.connectService();
    triggerShutdown_.connectService();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Motion::disconnectAllTaps()
{
    tapBrainStemStatus_.disconnectTap();
    tapOdometry_.disconnectTap();
    tapInitialPose_.disconnectTap();
    tapPlan_.disconnectTap();
    tapJoyAdapter_.disconnectTap();
    tapAps_.disconnectTap();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Motion::evaluateTriggers()
{
    if (triggerShutdown_.isShutdownRequested())
    {
        ros::shutdown();
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Motion::publishInformation()
{
    Pose<> currentPose = positionEstimator_.getPose();
    Velocity<> currentVelocity = positionEstimator_.getVelocity();

    geometry_msgs::Quaternion orientation = tf::createQuaternionMsgFromYaw(currentPose.theta);

    // Publish the TF of the pose
    geometry_msgs::TransformStamped messagePoseTf;
    messagePoseTf.header.frame_id = "odom";
    messagePoseTf.child_frame_id = "base_footprint";

    messagePoseTf.header.stamp = currentTime_;
    messagePoseTf.transform.translation.x = currentPose.x;
    messagePoseTf.transform.translation.y = currentPose.y;
    messagePoseTf.transform.translation.z = 0.0;
    messagePoseTf.transform.rotation = orientation;

    rosTfBroadcaster_.sendTransform(messagePoseTf);

    // Publish the required odometry for the planners
    nav_msgs::Odometry messagePose;
    messagePose.header.stamp = currentTime_;
    messagePose.header.frame_id = "odom";
    messagePose.child_frame_id = "base_footprint";

    // Position
    messagePose.pose.pose.position.x = currentPose.x;
    messagePose.pose.pose.position.y = currentPose.y;
    messagePose.pose.pose.position.z = 0.0;
    messagePose.pose.pose.orientation = orientation;

    // Velocity
    messagePose.twist.twist.linear.x = currentVelocity.linear;
    messagePose.twist.twist.linear.y = 0.0;
    messagePose.twist.twist.linear.z = 0.0;
    messagePose.twist.twist.angular.x = 0.0;
    messagePose.twist.twist.angular.y = 0.0;
    messagePose.twist.twist.angular.z = currentVelocity.angular;

    // Publish the Odometry
    pubOdom_.publish(messagePose);

    if (motionController_.newCommandAvailable())
    {
        Velocity<> command = motionController_.getExecutingCommand();

        geometry_msgs::Twist messageCmdVel;
        messageCmdVel.linear.x = command.linear;
        messageCmdVel.linear.y = 0;
        messageCmdVel.linear.z = 0;
        messageCmdVel.angular.x = 0;
        messageCmdVel.angular.y = 0;
        messageCmdVel.angular.z = command.angular;

        pubCmdVel_.publish(messageCmdVel);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Motion::scanTapsForData()
{
    bool commandGenerated = false;

    // Calculate the elapsed time
    double dT = Time::time2number(currentTime_) - Time::time2number(previousTime_);

    if (dT > 10 * (1.0/REFRESH_RATE_HZ))
    {
        ROS_INFO_STREAM("Skip to current time. Delta: " << dT);
        dT = 1.0 / REFRESH_RATE_HZ;
    }

    // If the joystick was touched, we know that we are latched
    if (tapJoyAdapter_.newDataAvailable())
    {
        if (tapJoyAdapter_.getLatchState())
        {
            commandUpdated_ = true;
            currentCommand_ = tapJoyAdapter_.getVelocity();
        }
    }

    // Provide the command to the position estimator
    if (tapOdometry_.newDataAvailable())
    {
        // Odometry<> odometry = tapOdometry_.getSensor()->getOdometry().velocity;
        Velocity<> velocity = tapOdometry_.getSensor()->getOdometry().velocity;

        positionEstimator_.run(dT, &velocity);
    }

    // Output the velocity command to the brainstem
    if (commandGenerated)
    {
        // If the brain stem is disconnected, simulate odometry
        // feeding the odometer the commanded velocity
        if (!tapBrainStemStatus_.isBrainStemConnected())
        {
            ROS_WARN_STREAM_ONCE_NAMED(rosNodeHandle_.getNamespace().c_str(),
                "Brainstem disconnected. Using simulated odometry");

            tapOdometry_.set(Time::time2number(ros::Time::now()),
                currentCommand_.linear,
                currentCommand_.angular);
        }

        ROS_INFO_STREAM("Sending command: " << currentCommand_);
        outputVelocityCommand(currentCommand_);
    }

    if (tapInitialPose_.newDataAvailable())
    {
        reset();
    }

//    // If there is a new plan to execute
//    if (tapPlan_.newDataAvailable())
//    {
//        trajectory_.clear();
//
//        Chuck chuck;
//        vector<SolutionNode<Grid2d>> solution = tapPlan_.getGoalPlan();
//
//        Trajectory trajectoryConverter(solution, chuck, 1.0 / REFRESH_RATE_HZ);
//        //trajectoryConverter.solution2trajectory(trajectory_);
//
//        for (auto milestone : trajectory_)
//        {
//            ROS_DEBUG_STREAM("Executing: " << milestone.first << " - " << milestone.second);
//        }
//
//        executionTime_ = 0.0;
//        nextScheduled_ = trajectory_.begin();
//        nextScheduledTime_ = nextScheduled_->first.arrivalTime;
//    }
}

} // namespace srs
