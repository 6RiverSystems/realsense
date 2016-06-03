#include <srsnode_motion/Motion.hpp>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>

#include <srslib_framework/math/Time.hpp>

#include <srslib_framework/graph/grid2d/Grid2d.hpp>
#include <srslib_framework/search/SearchPosition.hpp>
#include <srslib_framework/planning/pathplanning/SolutionNode.hpp>
#include <srslib_framework/robotics/robot/Chuck.hpp>
#include <srslib_framework/robotics/Odometry.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
Motion::Motion(string nodeName) :
    currentCommand_(),
    firstLocalization_(true),
    positionEstimator_(REFRESH_RATE_HZ),
    motionController_(),
    robot_(),
    rosNodeHandle_(nodeName),
    // tapPlan_(rosNodeHandle_),
    tapJoyAdapter_(rosNodeHandle_),
    tapBrainStem_(rosNodeHandle_),
    tapOdometry_(rosNodeHandle_),
    tapInitialPose_(rosNodeHandle_),
    tapAps_(rosNodeHandle_),
    triggerShutdown_(rosNodeHandle_),
    triggerStop_(rosNodeHandle_),
    trajectoryConverter_(robot_, 1.0 / 20.0 /*REFRESH_RATE_HZ*/), // #################

    tapCmdGoal_(rosNodeHandle_),
    tapMap_(rosNodeHandle_),
    currentGoal_()
{
    pubCmdVel_ = rosNodeHandle_.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
    pubOdometry_ = rosNodeHandle_.advertise<nav_msgs::Odometry>("odometry", 50);

    positionEstimator_.addSensor(tapOdometry_.getSensor());
    positionEstimator_.addSensor(tapAps_.getSensor());

    configServer_.setCallback(boost::bind(&Motion::onConfigChange, this, _1, _2));

    pubGoalPlan_ = rosNodeHandle_.advertise<nav_msgs::Path>("current_goal/plan", 1);
    pubGoalGoal_ = rosNodeHandle_.advertise<geometry_msgs::PoseStamped>("current_goal/goal", 1);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Motion::run()
{
    connectAllTaps();

    reset(tapInitialPose_.getPose());

    ros::Rate refreshRate(REFRESH_RATE_HZ);
    while (ros::ok())
    {
        ros::spinOnce();

        previousTime_ = currentTime_;
        currentTime_ = ros::Time::now();

        evaluateTriggers();
        scanTapsForData();
        stepNode();
        publishInformation();

        refreshRate.sleep();
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

////////////////////////////////////////////////////////////////////////////////////////////////////
void Motion::connectAllTaps()
{
    tapBrainStem_.connectTap();
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
    tapBrainStem_.disconnectTap();
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
void Motion::onConfigChange(MotionConfig& config, uint32_t level)
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
void Motion::outputVelocityCommand(Velocity<> outputCommand)
{
    geometry_msgs::Twist messageTwist;

    messageTwist.linear.x = outputCommand.linear;
    messageTwist.linear.y = 0;
    messageTwist.linear.z = 0;
    messageTwist.angular.x = 0;
    messageTwist.angular.y = 0;
    messageTwist.angular.z = 0.0; // outputCommand.angular; ######

    pubCmdVel_.publish(messageTwist);
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
void Motion::scanTapsForData()
{
    // If APS has reported a position and it is the first time that Motion
    // gets it, make sure that the Position Estimator knows of it.
    if (!tapAps_.hasNeverReported() && firstLocalization_)
    {
        ROS_INFO_STREAM("Reporting its first position: " << tapAps_.getPose());

        reset(tapAps_.getPose());
        firstLocalization_ = false;
    }

    if (tapInitialPose_.newDataAvailable())
    {
        ROS_INFO_STREAM("Received initial pose: " << tapInitialPose_.getPose());

        reset(tapInitialPose_.getPose());
        firstLocalization_ = false;
    }

    // If there is a new goal to reach
    if (tapCmdGoal_.newDataAvailable())
    {
        if (!tapJoyAdapter_.getLatchState())
        {
            executePlanToGoal(tapCmdGoal_.getGoal());
            publishGoal();
        }
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
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Motion::reset(Pose<> pose0)
{
    positionEstimator_.reset(pose0);
    motionController_.reset();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Motion::stepNode()
{
    bool commandGenerated = false;

    // Calculate the elapsed time
    double dT = Time::time2number(currentTime_) - Time::time2number(previousTime_);

    // If the joystick was touched, we know that we are latched
    if (tapJoyAdapter_.newDataAvailable())
    {
        // Stop the motion controller from whatever it was doing
        motionController_.stop(0);

        commandGenerated = true;
        currentCommand_ = tapJoyAdapter_.getVelocity();

//        ROS_DEBUG_STREAM_NAMED(rosNodeHandle_.getNamespace().c_str(),
//            "Receiving command from joystick: " << currentCommand_);
    }
    else {
        // Run the motion controller with the current pose estimate
        // only if the joystick is not latched
        if (!tapJoyAdapter_.getLatchState())
        {
            motionController_.run(dT, positionEstimator_.getPose());

            if (motionController_.newCommandAvailable())
            {
                currentCommand_ = motionController_.getExecutingCommand();
                commandGenerated = true;
            }
        }
    }

    // Provide the command to the position estimator
    if (tapOdometry_.newDataAvailable())
    {
        positionEstimator_.run(tapOdometry_.getSensor()->getOdometry());
    }

    // Output the velocity command to the brainstem
    if (commandGenerated)
    {
        // If the brain stem is disconnected, simulate odometry
        // feeding the odometer the commanded velocity
        if (!tapBrainStem_.isBrainStemConnected())
        {
            ROS_WARN_STREAM_ONCE_NAMED(rosNodeHandle_.getNamespace().c_str(),
                "Brainstem disconnected. Using simulated odometry");

            tapOdometry_.set(Time::time2number(ros::Time::now()),
                currentCommand_.linear,
                currentCommand_.angular);
        }

        ROS_INFO_STREAM_NAMED("Motion", "Sending command: " << currentCommand_);
        outputVelocityCommand(currentCommand_);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// TODO: This should be in Executive
void Motion::executePlanToGoal(Pose<> goal)
{
    currentGoal_ = goal;
    algorithm_.setGraph(tapMap_.getMap()->getGrid());

    int r1 = 19;
    int c1 = 29;
    Pose<> currentPose = positionEstimator_.getPose();
    //tapMap_.getMap()->getMapCoordinates(currentPose.x, currentPose.y, c1, r1);
    Grid2d::LocationType internalStart(c1, r1);
    int startAngle = Math::normalizeRad2deg90(currentPose.theta);

    int r2 = 19; //151;
    int c2 = 49; //52;
    //tapMap_.getMap()->getMapCoordinates(goal.x, goal.y, c2, r2);
    Grid2d::LocationType internalGoal(c2, r2);
    int goalAngle = Math::normalizeRad2deg90(goal.theta);

    ROS_INFO_STREAM("Looking for a path between " << currentPose << " (" <<
        c1 << "," << r1 << "," << startAngle
        << ") and " << goal << " (" << c2 << "," << r2 << "," << goalAngle << ")");

    algorithm_.search(
        SearchPosition<Grid2d>(internalStart, startAngle),
        SearchPosition<Grid2d>(internalGoal, goalAngle));

    vector<SolutionNode<Grid2d>> solution = algorithm_.getPath(tapMap_.getMap()->getResolution());

    if (!solution.empty())
    {
        Trajectory::TrajectoryType currentTrajectory_;

        trajectoryConverter_.calculateTrajectory(solution);
        trajectoryConverter_.getTrajectory(currentTrajectory_);
        motionController_.setTrajectory(currentTrajectory_);
    }
    else
    {
        ROS_INFO_STREAM("Path not found between " << currentPose << " (" <<
            c1 << "," << r1 << "," << startAngle
            << ") and " << goal << " (" << c2 << "," << r2 << "," << goalAngle << ")");
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Motion::publishGoal()
{
    vector<SolutionNode<Grid2d>> path = algorithm_.getPath(tapMap_.getMap()->getResolution());

    ros::Time planningTime = ros::Time::now();

    nav_msgs::Path messageGoalPlan;
    messageGoalPlan.header.frame_id = "map";
    messageGoalPlan.header.stamp = planningTime;

    vector<geometry_msgs::PoseStamped> planPoses;

    for (auto node : path)
    {
        geometry_msgs::PoseStamped poseStamped;
        tf::Quaternion quaternion = tf::createQuaternionFromYaw(node.pose.theta);

        poseStamped.pose.position.x = node.pose.x;
        poseStamped.pose.position.y = node.pose.y;
        poseStamped.pose.position.z = 0.0;
        poseStamped.pose.orientation.x = quaternion.x();
        poseStamped.pose.orientation.y = quaternion.y();
        poseStamped.pose.orientation.z = quaternion.z();
        poseStamped.pose.orientation.w = quaternion.w();

        planPoses.push_back(poseStamped);
    }

    messageGoalPlan.poses = planPoses;

    geometry_msgs::PoseStamped messageGoal;
    tf::Quaternion quaternion = tf::createQuaternionFromYaw(currentGoal_.theta);

    messageGoal.header.stamp = planningTime;
    messageGoal.pose.position.x = currentGoal_.x;
    messageGoal.pose.position.y = currentGoal_.y;
    messageGoal.pose.position.z = 0.0;
    messageGoal.pose.orientation.x = quaternion.x();
    messageGoal.pose.orientation.y = quaternion.y();
    messageGoal.pose.orientation.z = quaternion.z();
    messageGoal.pose.orientation.w = quaternion.w();

    pubGoalGoal_.publish(messageGoal);
    pubGoalPlan_.publish(messageGoalPlan);
}

} // namespace srs
