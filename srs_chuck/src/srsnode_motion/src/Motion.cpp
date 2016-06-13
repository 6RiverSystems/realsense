#include <srsnode_motion/Motion.hpp>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>

#include <srslib_framework/math/AngleMath.hpp>

#include <srslib_framework/graph/grid2d/Grid2d.hpp>
#include <srslib_framework/search/SearchPosition.hpp>
#include <srslib_framework/planning/pathplanning/Solution.hpp>
#include <srslib_framework/robotics/robot/Chuck.hpp>
#include <srslib_framework/robotics/Odometry.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
Motion::Motion(string nodeName) :
    robot_(),
    currentCommand_(),
    firstLocalization_(true),
    rosNodeHandle_(nodeName),
    positionEstimator_(1.0 / REFRESH_RATE_HZ),
    motionController_(1.0 / REFRESH_RATE_HZ),
    // tapPlan_(rosNodeHandle_),
    tapJoyAdapter_(rosNodeHandle_),
    tapBrainStem_(rosNodeHandle_),
    tapOdometry_(rosNodeHandle_),
    tapInitialPose_(rosNodeHandle_),
    tapAps_(rosNodeHandle_),
    triggerShutdown_(rosNodeHandle_),
    triggerStop_(rosNodeHandle_),
    tapCmdGoal_(rosNodeHandle_),
    tapMap_(rosNodeHandle_)
{
    motionController_.setRobot(robot_);

    pubOdometry_ = rosNodeHandle_.advertise<nav_msgs::Odometry>("odometry", 50);

    positionEstimator_.addSensor(tapAps_.getSensor());

    configServer_.setCallback(boost::bind(&Motion::onConfigChange, this, _1, _2));

    pubGoalPlan_ = rosNodeHandle_.advertise<nav_msgs::Path>("current_goal/plan", 1);
    pubGoalGoal_ = rosNodeHandle_.advertise<geometry_msgs::PoseStamped>("current_goal/goal", 1);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Motion::run()
{
    connectAllTaps();

    // Establish an "acceptable" initial position
    tapInitialPose_.set(0, 3.0, 3.0, 0);
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

    robot_.goalReachedDistance = configuration_.goal_reached_distance;
    ROS_INFO_STREAM("Goal reached distance [m]: " << configuration_.goal_reached_distance);

    robot_.maxAngularAcceleration = configuration_.max_angular_acceleration;
    ROS_INFO_STREAM("Max angular acceleration [m/s^2]: " << configuration_.max_angular_acceleration);

    robot_. maxAngularVelocity = configuration_.max_angular_velocity;
    ROS_INFO_STREAM("Max angular velocity [m/s]: " << configuration_.max_angular_velocity);

    robot_.maxLinearAcceleration = configuration_.max_linear_acceleration;
    ROS_INFO_STREAM("Max linear acceleration [m/s^2]: " << configuration_.max_linear_acceleration);

    robot_.maxLinearVelocity = configuration_.max_linear_velocity;
    ROS_INFO_STREAM("Max linear velocity [m/s]: " << configuration_.max_linear_velocity);

    robot_.maxLookAheadDistance = configuration_.max_look_ahead_distance;
    ROS_INFO_STREAM("Max look-ahead distance [m]: " << configuration_.max_look_ahead_distance);

    robot_.minLookAheadDistance = configuration_.min_look_ahead_distance;
    ROS_INFO_STREAM("Min look-ahead distance [m]: " << configuration_.min_look_ahead_distance);

    robot_.ratioLookAheadDistance = configuration_.ratio_look_ahead_distance;
    ROS_INFO_STREAM("Ratio look-ahead distance []: " << configuration_.ratio_look_ahead_distance);

    robot_.travelAngularAcceleration = configuration_.travel_angular_acceleration;
    ROS_INFO_STREAM("Travel angular acceleration [rad/s^2]: " << configuration_.travel_angular_acceleration);

    robot_.travelAngularVelocity = configuration_.travel_angular_velocity;
    ROS_INFO_STREAM("Travel angular velocity [rad/s]: " << configuration_.travel_angular_velocity);

    robot_.travelLinearAcceleration = configuration_.travel_linear_acceleration;
    ROS_INFO_STREAM("Travel linear acceleration [m/s^2]: " << configuration_.travel_linear_acceleration);

    robot_.travelLinearVelocity = configuration_.travel_linear_velocity;
    ROS_INFO_STREAM("Travel linear velocity [m/s]: " << configuration_.travel_linear_velocity);

    robot_.travelRotationVelocity = configuration_.travel_rotation_velocity;
    ROS_INFO_STREAM("Travel rotation velocity [rad/s]: " << configuration_.travel_rotation_velocity);

    tapOdometry_.getSensor()->enable(configuration_.odometry_enabled);
    ROS_INFO_STREAM("Odometry sensor enabled: " << configuration_.odometry_enabled);

    motionController_.setRobot(robot_);
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

    // if (!motionController_.isMoving() && tapMap_.getMap() && !motionController_.isGoalReached())
    // {
    //     executePlanToGoal(tapCmdGoal_.getGoal());
    //     publishGoal();
    // }

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
    bool odometryAvailable = tapOdometry_.newDataAvailable();
    Odometry<> currentOdometry = tapOdometry_.getSensor()->getOdometry();

    Velocity<> currentJoystickCommand = tapJoyAdapter_.getVelocity();

    // If the joystick was touched, we know that we are latched
    if (tapJoyAdapter_.newDataAvailable())
    {
        // If the joystick was touched, we know that we are latched
        if (tapJoyAdapter_.getLatchState())
        {
            // Tell the motion controller that the joystick is
            // assuming control
            motionController_.switchToManual();
        }
        else {
            // Tell the motion controller that the joystick is
            // assuming control
            motionController_.switchToAutonomous();
        }
    }

    // Provide the command to the position estimator
    if (odometryAvailable)
    {
        positionEstimator_.run(&currentOdometry);
    }
    Pose<> currentPose = positionEstimator_.getPose();

    motionController_.run(currentPose, currentOdometry, currentJoystickCommand);

    if (motionController_.isMoving())
    {
        currentCommand_ = motionController_.getExecutingCommand();

        // If the brain stem is disconnected and the motion controller is moving, simulate odometry
        // feeding the odometer with the commanded velocity
        if (!tapBrainStem_.isBrainStemConnected())
        {
            ROS_WARN_STREAM_ONCE_NAMED(rosNodeHandle_.getNamespace().c_str(),
                "Brainstem disconnected. Using simulated odometry");

            simulatedT_ += 1.0 / REFRESH_RATE_HZ;
            tapOdometry_.set(simulatedT_,
                currentCommand_.linear,
                currentCommand_.angular);
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// TODO: This should be in Executive
void Motion::executePlanToGoal(Pose<> goalPose)
{
    Pose<> robotPose = positionEstimator_.getPose();

    algorithm_.setGraph(tapMap_.getMap()->getGrid());

    int fromR = 0;
    int fromC = 0;
    tapMap_.getMap()->getMapCoordinates(robotPose.x, robotPose.y, fromC, fromR);
    Grid2d::LocationType internalStart(fromC, fromR);
    int startAngle = AngleMath::normalizeRad2deg90(robotPose.theta);

    int toR = 0;
    int toC = 0;
    tapMap_.getMap()->getMapCoordinates(goalPose.x, goalPose.y, toC, toR);
    Grid2d::LocationType internalGoal(toC, toR);
    int goalAngle = AngleMath::normalizeRad2deg90(goalPose.theta);

    ROS_DEBUG_STREAM_NAMED("Motion", "Looking for a path between " << robotPose << " (" <<
        fromC << "," << fromR << "," << startAngle
        << ") and " << goalPose << " (" << toC << "," << toR << "," << goalAngle << ")");

    algorithm_.search(
        SearchPosition<Grid2d>(internalStart, startAngle),
        SearchPosition<Grid2d>(internalGoal, goalAngle));

    Solution<Grid2d> solution = algorithm_.getSolution(tapMap_.getMap()->getResolution());

    if (!solution.empty())
    {
        motionController_.execute(solution);
        simulatedT_ = 0.0;
    }
    else
    {
        ROS_ERROR_STREAM_NAMED("Motion", "Path not found between " <<
            robotPose << " (" << fromC << "," << fromR << "," << startAngle << ") and " <<
            goalPose << " (" << toC << "," << toR << "," << goalAngle << ")");
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Motion::publishGoal()
{
    Pose<> goal = motionController_.getGoal();

    Solution<Grid2d> solution = algorithm_.getSolution(tapMap_.getMap()->getResolution());

    ros::Time planningTime = ros::Time::now();

    nav_msgs::Path messageGoalPlan;
    messageGoalPlan.header.frame_id = "map";
    messageGoalPlan.header.stamp = planningTime;

    vector<geometry_msgs::PoseStamped> planPoses;

    for (auto solutionNode : solution)
    {
        geometry_msgs::PoseStamped poseStamped;
        tf::Quaternion quaternion = tf::createQuaternionFromYaw(solutionNode.toPose.theta);

        poseStamped.pose.position.x = solutionNode.toPose.x;
        poseStamped.pose.position.y = solutionNode.toPose.y;
        poseStamped.pose.position.z = 0.0;
        poseStamped.pose.orientation.x = quaternion.x();
        poseStamped.pose.orientation.y = quaternion.y();
        poseStamped.pose.orientation.z = quaternion.z();
        poseStamped.pose.orientation.w = quaternion.w();

        planPoses.push_back(poseStamped);
    }

    messageGoalPlan.poses = planPoses;

    geometry_msgs::PoseStamped messageGoal;
    tf::Quaternion quaternion = tf::createQuaternionFromYaw(goal.theta);

    messageGoal.header.stamp = planningTime;
    messageGoal.pose.position.x = goal.x;
    messageGoal.pose.position.y = goal.y;
    messageGoal.pose.position.z = 0.0;
    messageGoal.pose.orientation.x = quaternion.x();
    messageGoal.pose.orientation.y = quaternion.y();
    messageGoal.pose.orientation.z = quaternion.z();
    messageGoal.pose.orientation.w = quaternion.w();

    pubGoalGoal_.publish(messageGoal);
    pubGoalPlan_.publish(messageGoalPlan);
}

} // namespace srs
