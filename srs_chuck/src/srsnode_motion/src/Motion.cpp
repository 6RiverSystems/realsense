#include <srsnode_motion/Motion.hpp>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#include <srslib_framework/math/Time.hpp>

#include <srslib_framework/graph/grid2d/Grid2d.hpp>
#include <srslib_framework/search/SearchPosition.hpp>
#include <srslib_framework/planning/pathplanning/SolutionNode.hpp>
#include <srslib_framework/robotics/robot/Chuck.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
Motion::Motion(string nodeName) :
    commandUpdated_(false),
    currentCommand_(),
    firstLocalization_(true),
    positionEstimator_(),
    motionController_(),
    robot_(),
    rosNodeHandle_(nodeName),
    // tapPlan_(rosNodeHandle_),
    tapJoyAdapter_(rosNodeHandle_),
    tapBrainStemStatus_(rosNodeHandle_),
    tapOdometry_(rosNodeHandle_),
    tapInitialPose_(rosNodeHandle_),
    tapAps_(rosNodeHandle_),
    triggerShutdown_(rosNodeHandle_),
    triggerStop_(rosNodeHandle_),
    trajectoryConverter_(robot_, 1.0 / REFRESH_RATE_HZ),
    tapCmdGoal_(rosNodeHandle_),
    tapMap_(rosNodeHandle_)
{
    pubCmdVel_ = rosNodeHandle_.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
    pubOdometry_ = rosNodeHandle_.advertise<nav_msgs::Odometry>("odometry", 50);

    positionEstimator_.addSensor(tapOdometry_.getSensor());
    positionEstimator_.addSensor(tapAps_.getSensor());

    configServer_.setCallback(boost::bind(&Motion::onConfigChange, this, _1, _2));
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Motion::reset()
{
    commandUpdated_ = false;
    currentCommand_ = Velocity<>();

    positionEstimator_.reset(tapInitialPose_.getPose());
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

        // TODO: For now the position estimator ignores the command
        Velocity<>* command = commandUpdated_ ? &currentCommand_ : nullptr;
        positionEstimator_.run(dT, nullptr);

        if (!triggerStop_.isTriggerRequested())
        {
            motionController_.run(dT, positionEstimator_.getPose());
        }
        else
        {
            motionController_.stop(0);
        }

        if (motionController_.newCommandAvailable())
        {
            sendVelocityCommand(motionController_.getExecutingCommand());
        }

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
    //tapPlan_.connectTap();
    tapJoyAdapter_.connectTap();
    tapAps_.connectTap();

    triggerStop_.connectService();
    triggerShutdown_.connectService();

    tapCmdGoal_.connectTap();
    tapMap_.connectTap();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Motion::disconnectAllTaps()
{
    tapBrainStemStatus_.disconnectTap();
    tapOdometry_.disconnectTap();
    tapInitialPose_.disconnectTap();
    //tapPlan_.disconnectTap();
    tapJoyAdapter_.disconnectTap();
    tapAps_.disconnectTap();

    tapCmdGoal_.disconnectTap();
    tapMap_.disconnectTap();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Motion::evaluateTriggers()
{
    if (triggerShutdown_.isTriggerRequested())
    {
        ros::shutdown();
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Motion::onConfigChange(DynamicConfig& config, uint32_t level)
{
    configuration_ = config;

    tapAps_.getSensor()->enable(configuration_.aps_enabled);
    ROS_INFO_STREAM("APS sensor enabled: " << configuration_.aps_enabled);

    tapOdometry_.getSensor()->enable(configuration_.odometry_enabled);
    ROS_INFO_STREAM("Odometry sensor enabled: " << configuration_.odometry_enabled);

    motionController_.setLookAhead(configuration_.look_ahead);
    ROS_INFO_STREAM("Motion controller look-ahead: " << configuration_.look_ahead);

    trajectoryConverter_.setMinimumVelocity(configuration_.min_velocity);
    ROS_INFO_STREAM("Motion controller minimum velocity: " << configuration_.min_velocity);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Motion::publishInformation()
{
    Pose<> currentPose = positionEstimator_.getPose();
    Velocity<> currentVelocity = positionEstimator_.getVelocity();

    geometry_msgs::Quaternion orientation = tf::createQuaternionMsgFromYaw(currentPose.theta);

    // Publish the TF of the pose
    geometry_msgs::TransformStamped messagePoseTf;
    messagePoseTf.header.frame_id = "srsnode_motion/odometry";
    messagePoseTf.child_frame_id = "base_footprint";

    messagePoseTf.header.stamp = currentTime_;
    messagePoseTf.transform.translation.x = currentPose.x;
    messagePoseTf.transform.translation.y = currentPose.y;
    messagePoseTf.transform.translation.z = 0.0;
    messagePoseTf.transform.rotation = orientation;

    rosTfBroadcaster_.sendTransform(messagePoseTf);

    // Publish the required odometry for the planners
    nav_msgs::Odometry messageOdometry;
    messageOdometry.header.stamp = currentTime_;
    messageOdometry.header.frame_id = "srsnode_motion/odometry";
    messageOdometry.child_frame_id = "base_footprint";

    // Position
    messageOdometry.pose.pose.position.x = currentPose.x;
    messageOdometry.pose.pose.position.y = currentPose.y;
    messageOdometry.pose.pose.position.z = 0.0;
    messageOdometry.pose.pose.orientation = orientation;

    // Velocity
    messageOdometry.twist.twist.linear.x = currentVelocity.linear;
    messageOdometry.twist.twist.linear.y = 0.0;
    messageOdometry.twist.twist.linear.z = 0.0;
    messageOdometry.twist.twist.angular.x = 0.0;
    messageOdometry.twist.twist.angular.y = 0.0;
    messageOdometry.twist.twist.angular.z = currentVelocity.angular;

    // Publish the Odometry
    pubOdometry_.publish(messageOdometry);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Motion::sendVelocityCommand(Velocity<> command)
{
    geometry_msgs::Twist messageTwist;

    messageTwist.linear.x = command.linear;
    messageTwist.linear.y = 0;
    messageTwist.linear.z = 0;
    messageTwist.angular.x = 0;
    messageTwist.angular.y = 0;
    messageTwist.angular.z = command.angular;

    pubCmdVel_.publish(messageTwist);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Motion::scanTapsForData()
{
    // If APS has reported a position and it is the first time that Motion
    // gets it, make sure that the Position Estimator knows of it.
    if (!tapAps_.hasNeverReported() && firstLocalization_)
    {
        positionEstimator_.reset(tapAps_.getPose());
        firstLocalization_ = false;

        ROS_INFO_STREAM("Reporting its first position " << tapAps_.getPose());
    }

    // If there is another source of command velocities, follow that request
    if (tapJoyAdapter_.newDataAvailable())
    {
        if (tapJoyAdapter_.getLatchState())
        {
            commandUpdated_ = true;
            currentCommand_ = tapJoyAdapter_.getVelocity();
        }
    }

    // If the brain stem is disconnected, simulate odometry
    // feeding the odometer the commanded velocity
    if (!tapBrainStemStatus_.isBrainStemConnected())
    {
        tapOdometry_.set(Time::time2number(ros::Time::now()),
            currentCommand_.linear,
            currentCommand_.angular);
    }

    if (tapInitialPose_.newDataAvailable())
    {
        // If we are given an initial position,
        // we won't need to wait for a first localization
        firstLocalization_ = false;
        reset();

    }

//    // If there is a new plan to execute
//    if (tapPlan_.newDataAvailable())
//    {
//        vector<SolutionNode<Grid2d>> solution = tapPlan_.getPlan();
//        Trajectory::TrajectoryType currentTrajectory_;
//
//        trajectoryConverter_.calculateTrajectory(solution);
//        trajectoryConverter_.getTrajectory(currentTrajectory_);
//        motionController_.setTrajectory(currentTrajectory_);
//    }

    // If there is a new goal to reach
    if (tapCmdGoal_.newDataAvailable())
    {
        executePlanToGoal(tapCmdGoal_.getGoal());
    }
}


////////////////////////////////////////////////////////////////////////////////////////////////////
// TODO: This should be in Executive
void Motion::executePlanToGoal(Pose<> goal)
{
     algorithm_.setGraph(tapMap_.getMap()->getGrid());

    int r = 0;
    int c = 0;

    Pose<> currentPose = positionEstimator_.getPose();
    tapMap_.getMap()->getMapCoordinates(currentPose.x, currentPose.y, c, r);
    Grid2d::LocationType internalStart(c, r);

    // tapMap_.getMap()->getMapCoordinates(goal.x, goal.y, c, r);
    Grid2d::LocationType internalGoal(c + 10, r);

    algorithm_.search(
        SearchPosition<Grid2d>(internalStart, 0),
        SearchPosition<Grid2d>(internalGoal, 0));

    vector<SolutionNode<Grid2d>> solution = algorithm_.getPath(tapMap_.getMap()->getResolution());
    Trajectory::TrajectoryType currentTrajectory_;

    trajectoryConverter_.calculateTrajectory(solution);
    trajectoryConverter_.getTrajectory(currentTrajectory_);
    motionController_.setTrajectory(currentTrajectory_);
}

} // namespace srs
