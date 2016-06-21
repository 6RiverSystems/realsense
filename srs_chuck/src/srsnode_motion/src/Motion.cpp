#include <srsnode_motion/Motion.hpp>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>
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
    firstLocalization_(true),
    isJoystickLatched_(false),
    rosNodeHandle_(nodeName),
    positionEstimator_(1.0 / REFRESH_RATE_HZ),
    motionController_(1.0 / REFRESH_RATE_HZ),
    pingDecimator_(0),
    simulatedT_(0.0)
{
    motionController_.setRobot(robot_);

    positionEstimator_.addSensor(tapAps_.getSensor());

    configServer_.setCallback(boost::bind(&Motion::onConfigChange, this, _1, _2));

    pubOdometry_ = rosNodeHandle_.advertise<nav_msgs::Odometry>(
        "/internal/sensors/odometry/velocity", 50);

    pubStatusGoalPlan_ = rosNodeHandle_.advertise<nav_msgs::Path>(
        "/internal/state/current_goal/plan", 1);
    pubStatusGoalGoal_ = rosNodeHandle_.advertise<geometry_msgs::PoseStamped>(
        "/internal/state/current_goal/goal", 1);
    pubStatusGoalArrived_ = rosNodeHandle_.advertise<std_msgs::Bool>(
        "/internal/state/current_goal/arrived", 1);
    pubPing_ = rosNodeHandle_.advertise<std_msgs::Bool>(
        "/internal/state/ping", 1);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Motion::run()
{
    connectAllTaps();

    currentTime_ = ros::Time::now();

    // Establish an "acceptable" initial position, at least until the
    // Position Estimator can get its bearings
    tapInitialPose_.set(TimeMath::time2number(currentTime_), 3.0, 3.0, 0);
    reset(tapInitialPose_.getPose());

    ros::Rate refreshRate(REFRESH_RATE_HZ);
    while (ros::ok())
    {
        ros::spinOnce();

        previousTime_ = currentTime_;
        currentTime_ = ros::Time::now();

        evaluateTriggers();
        scanTapsForData();
        publishArrived();

        stepNode();

        publishOdometry();

        publishPing();

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
    tapJoyAdapter_.connectTap();
    tapAps_.connectTap();
    tapInternalGoal_.connectTap();
    tapMap_.connectTap();

    triggerStop_.connectService();
    triggerShutdown_.connectService();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Motion::disconnectAllTaps()
{
    tapBrainStem_.disconnectTap();
    tapOdometry_.disconnectTap();
    tapInitialPose_.disconnectTap();
    tapJoyAdapter_.disconnectTap();
    tapAps_.disconnectTap();
    tapInternalGoal_.disconnectTap();
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

    robot_.adaptiveLookAhead = configuration_.adaptive_lookahead_enabled;
    ROS_INFO_STREAM_NAMED("Motion", "Adaptive look-ahead enabled [t/f]: " <<
        configuration_.adaptive_lookahead_enabled);

    tapAps_.getSensor()->enable(configuration_.aps_enabled);
    ROS_INFO_STREAM_NAMED("Motion", "APS sensor enabled: " <<
        configuration_.aps_enabled);

    robot_.goalReachedDistance = configuration_.goal_reached_distance;
    ROS_INFO_STREAM_NAMED("Motion", "Goal reached distance [m]: " <<
        configuration_.goal_reached_distance);

    robot_.goalReachedAngle = configuration_.goal_reached_angle;
    ROS_INFO_STREAM_NAMED("Motion", "Goal reached angle [rad]: " <<
        configuration_.goal_reached_angle);

    robot_.maxAngularAcceleration = configuration_.max_angular_acceleration;
    ROS_INFO_STREAM_NAMED("Motion", "Max angular acceleration [m/s^2]: " <<
        configuration_.max_angular_acceleration);

    robot_.maxAngularVelocity = configuration_.max_angular_velocity;
    ROS_INFO_STREAM_NAMED("Motion", "Max angular velocity [m/s]: " <<
        configuration_.max_angular_velocity);

    robot_.maxLinearAcceleration = configuration_.max_linear_acceleration;
    ROS_INFO_STREAM_NAMED("Motion", "Max linear acceleration [m/s^2]: " <<
        configuration_.max_linear_acceleration);

    robot_.maxLinearVelocity = configuration_.max_linear_velocity;
    ROS_INFO_STREAM_NAMED("Motion", "Max linear velocity [m/s]: " <<
        configuration_.max_linear_velocity);

    robot_.maxLookAheadDistance = configuration_.max_look_ahead_distance;
    ROS_INFO_STREAM_NAMED("Motion", "Max look-ahead distance [m]: " <<
        configuration_.max_look_ahead_distance);

    robot_.minAngularVelocity = configuration_.min_angular_velocity;
    ROS_INFO_STREAM_NAMED("Motion", "Minimum angular velocity during motion [rad/s]: " <<
        configuration_.min_angular_velocity);

    robot_.minLinearVelocity = configuration_.min_linear_velocity;
    ROS_INFO_STREAM_NAMED("Motion", "Minimum linear velocity during motion [m/s]: " <<
        configuration_.min_linear_velocity);

    robot_.minLookAheadDistance = configuration_.min_look_ahead_distance;
    ROS_INFO_STREAM_NAMED("Motion", "Min look-ahead distance [m]: " <<
        configuration_.min_look_ahead_distance);

    robot_.minPhysicalAngularVelocity = configuration_.min_physical_angular_velocity;
    ROS_INFO_STREAM_NAMED("Motion", "Minimum physical angular velocity [rad/s]: " <<
        configuration_.min_physical_angular_velocity);

    robot_.minPhysicalLinearVelocity = configuration_.min_physical_linear_velocity;
    ROS_INFO_STREAM_NAMED("Motion", "Minimum physical linear velocity [m/s]: " <<
        configuration_.min_physical_linear_velocity);

    robot_.ratioCrawl = configuration_.ratio_crawl;
    ROS_INFO_STREAM_NAMED("Motion", "Ratio of the dynamic motion controller in crawl mode []: " <<
        configuration_.ratio_crawl);

    robot_.ratioManual = configuration_.ratio_manual;
    ROS_INFO_STREAM_NAMED("Motion", "Ratio of the dynamic motion controller in manual mode []: " <<
        configuration_.ratio_manual);

    robot_.travelAngularAcceleration = configuration_.travel_angular_acceleration;
    ROS_INFO_STREAM_NAMED("Motion", "Travel angular acceleration [rad/s^2]: " <<
        configuration_.travel_angular_acceleration);

    robot_.travelAngularVelocity = configuration_.travel_angular_velocity;
    ROS_INFO_STREAM_NAMED("Motion", "Travel angular velocity [rad/s]: " <<
        configuration_.travel_angular_velocity);

    robot_.travelCurveZoneRadius = configuration_.travel_curve_zone_radius;
    ROS_INFO_STREAM_NAMED("Motion", "Travel reduced velocity radius during curves [m]: " <<
        configuration_.travel_curve_zone_radius);

    robot_.travelCurvingVelocity = configuration_.travel_curving_linear_velocity;
    ROS_INFO_STREAM_NAMED("Motion", "Travel linear velocity during curves [m/s]: " <<
        configuration_.travel_curving_linear_velocity);

    robot_.travelLinearAcceleration = configuration_.travel_linear_acceleration;
    ROS_INFO_STREAM_NAMED("Motion", "Travel linear acceleration [m/s^2]: " <<
        configuration_.travel_linear_acceleration);

    robot_.travelLinearVelocity = configuration_.travel_linear_velocity;
    ROS_INFO_STREAM_NAMED("Motion", "Travel linear velocity [m/s]: " <<
        configuration_.travel_linear_velocity);

    robot_.travelRotationVelocity = configuration_.travel_rotation_velocity;
    ROS_INFO_STREAM_NAMED("Motion", "Travel rotation velocity [rad/s]: " <<
        configuration_.travel_rotation_velocity);

    robot_.zeroLookAheadDistance = configuration_.zero_look_ahead_distance;
    ROS_INFO_STREAM_NAMED("Motion", "Zero-point motion controller look-ahead distance [m]: " <<
        configuration_.zero_look_ahead_distance);

    motionController_.setRobot(robot_);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Motion::publishArrived()
{
    // Publish this only if the motion controller says that
    // something has changed
    if (motionController_.hasArrivedChanged())
    {
        std_msgs::Bool messageGoalArrived;

        if (motionController_.hasArrived())
        {
            messageGoalArrived.data = true;
            ROS_DEBUG_STREAM_NAMED("Motion", "Arrived at goal: " <<
                motionController_.getFinalGoal());
        }
        else
        {
            messageGoalArrived.data = false;
            ROS_DEBUG_STREAM_NAMED("Motion", "Departed for goal: " <<
                motionController_.getFinalGoal());
        }

        pubStatusGoalArrived_.publish(messageGoalArrived);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Motion::publishGoal()
{
    ros::Time planningTime = ros::Time::now();

    Pose<> goal = motionController_.getFinalGoal();

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

    pubStatusGoalGoal_.publish(messageGoal);

    Solution<Grid2d> solution = algorithm_.getSolution(tapMap_.getMap()->getResolution());

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

    pubStatusGoalPlan_.publish(messageGoalPlan);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Motion::publishOdometry()
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
void Motion::publishPing()
{
	constexpr uint32_t frequencyDivisor = uint32_t(REFRESH_RATE_HZ / PING_HZ);

	if ((pingDecimator_ % frequencyDivisor) == 0)
	{
		std_msgs::Bool messagePing;
		messagePing.data = true;

		pubPing_.publish(messagePing);
	}

    pingDecimator_++;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Motion::scanTapsForData()
{
    // If APS has reported a position and it is the first time that Motion
    // gets it, make sure that the Position Estimator knows of it.
    if (!tapAps_.hasNeverReported() && firstLocalization_)
    {
        ROS_DEBUG_STREAM("Reporting its first position: " << tapAps_.getPose());

        reset(tapAps_.getPose());
        firstLocalization_ = false;
    }

    if (tapInitialPose_.newDataAvailable())
    {
        ROS_DEBUG_STREAM("Received initial pose: " << tapInitialPose_.getPose());

        reset(tapInitialPose_.getPose());
        firstLocalization_ = false;
    }

    // If the joystick was touched, read the state of the latch
    if (tapJoyAdapter_.newDataAvailable())
    {
        // If the controller emergency button has been pressed, tell
        // the motion controller immediately
        if (tapJoyAdapter_.getEmergencyState())
        {
            motionController_.emergencyStop();
        }

        // Store the latch state for later use
        isJoystickLatched_ = tapJoyAdapter_.getLatchState();
    }

    // if (!motionController_.isMoving() && tapMap_.getMap() && !motionController_.isGoalReached())
    // {
    //     executePlanToGoal(tapCmdGoal_.getGoal());
    //     publishGoal();
    // }

    // If there is a new goal to reach
    if (tapInternalGoal_.newDataAvailable())
    {
        if (!isJoystickLatched_)
        {
            executePlanToGoal(tapInternalGoal_.getPose());
            publishGoal();
        }
    }

    // Check if odometry is available
    isOdometryAvailable_ = tapOdometry_.newDataAvailable();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Motion::reset(Pose<> pose0)
{
    positionEstimator_.reset(pose0);
    motionController_.reset();
    simulatedT_ = 0.0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Motion::stepNode()
{
    Velocity<> currentJoystickCommand = tapJoyAdapter_.getVelocity();

    if (motionController_.isEmergencyDeclared())
    {
        // If an emergency has been declared, reset the motion controller
        // only once the joystick has been unlatched
        if (!isJoystickLatched_)
        {
            motionController_.reset();
            motionController_.switchToAutonomous();
        }
    }
    else
    {
        // Depending on the state of the joystick latch, enable or disable
        // the autonomous mode
        if (isJoystickLatched_)
        {
            motionController_.switchToManual();
        }
        else
        {
            motionController_.switchToAutonomous();
        }
    }

    Odometry<> currentOdometry = tapOdometry_.getOdometry();

    // Provide the command to the position estimator if available
    positionEstimator_.run(isOdometryAvailable_ ? &currentOdometry : nullptr);
    Pose<> currentPose = positionEstimator_.getPose();

    // Run the motion controller
    motionController_.run(currentPose, currentOdometry, currentJoystickCommand);

    // If the brain stem is disconnected and the motion controller is moving, simulate odometry
    // feeding the odometer with the commanded velocity
    if (!tapBrainStem_.isBrainStemConnected())
    {
        ROS_WARN_STREAM_ONCE_NAMED(rosNodeHandle_.getNamespace().c_str(),
            "Brainstem disconnected. Using simulated odometry");

        Velocity<> currentCommand = motionController_.getExecutingCommand();

        simulatedT_ += 1.0 / REFRESH_RATE_HZ;
        tapOdometry_.set(simulatedT_,
            currentCommand.linear,
            currentCommand.angular);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// TODO: should this be in Executive
void Motion::executePlanToGoal(Pose<> goalPose)
{
    Pose<> robotPose = positionEstimator_.getPose();

    algorithm_.setGraph(tapMap_.getMap()->getGrid());

    // Prepare the start position for the search
    int fromR = 0;
    int fromC = 0;
    tapMap_.getMap()->getMapCoordinates(robotPose.x, robotPose.y, fromC, fromR);
    Grid2d::LocationType internalStart(fromC, fromR);
    int startAngle = AngleMath::normalizeRad2deg90(robotPose.theta);

    // Keep the goal in line with current robot pose
    goalPose.x = (abs(goalPose.x - robotPose.x) < 0.4) ? robotPose.x : goalPose.x;
    goalPose.y = (abs(goalPose.y - robotPose.y) < 0.4) ? robotPose.y : goalPose.y;

    // Prepare the goal position for the search
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
        ROS_DEBUG_STREAM_NAMED("Motion", "Found solution: " << endl << solution);

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

} // namespace srs
