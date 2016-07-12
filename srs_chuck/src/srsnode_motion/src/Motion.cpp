#include <srsnode_motion/Motion.hpp>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <tf/transform_broadcaster.h>

#include <srslib_framework/MsgPose.h>

#include <srslib_framework/math/AngleMath.hpp>

#include <srslib_framework/planning/pathplanning/Solution.hpp>

#include <srslib_framework/robotics/robot/Chuck.hpp>
#include <srslib_framework/robotics/Odometry.hpp>

#include <srslib_framework/ros/message/PoseMessageFactory.hpp>

#include <srsnode_motion/tap/aps/FactoryApsNoise.hpp>
#include <srsnode_motion/tap/odometry/FactoryOdometryNoise.hpp>

#include <srsnode_motion/MotionConfig.h>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
Motion::Motion(string nodeName) :
    isApsAvailable_(false),
    isCustomActionEnabled_(false),
    isJoystickLatched_(false),
    rosNodeHandle_(nodeName),
    pingTimer_(),
    positionEstimator_(1.0 / REFRESH_RATE_HZ),
    motionController_(1.0 / REFRESH_RATE_HZ),
    simulatedT_(0.0)
{
    positionEstimator_.addSensor(tapAps_.getSensor());

    configServer_.setCallback(boost::bind(&Motion::onConfigChange, this, _1, _2));

    pubOdometry_ = rosNodeHandle_.advertise<nav_msgs::Odometry>(
        "/internal/sensors/odometry/velocity", 50);

    pubPing_ = rosNodeHandle_.advertise<std_msgs::Bool>(
        "/internal/state/ping", 1);

    pubRobotAccOdometry_ = rosNodeHandle_.advertise<srslib_framework::MsgPose>(
        "/internal/state/robot/acc_odometry", 100);
    pubRobotPose_ = rosNodeHandle_.advertise<srslib_framework::MsgPose>(
        "/internal/state/robot/pose", 100);
    pubRobotLocalized_ = rosNodeHandle_.advertise<std_msgs::Bool>(
        "/internal/state/robot/localized", 1, true);

    pubStatusGoalArrived_ = rosNodeHandle_.advertise<std_msgs::Bool>(
        "/internal/state/goal/arrived", 1);
    pubStatusGoalLanding_ = rosNodeHandle_.advertise<geometry_msgs::PolygonStamped>(
        "/internal/state/goal/landing", 1);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Motion::run()
{
    connectAllTaps();

    currentTime_ = ros::Time::now();

    // Start the ping timer
    pingTimer_ = rosNodeHandle_.createTimer(ros::Duration(1.0 / PING_HZ),
        boost::bind(&Motion::pingCallback, this, _1));

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
        stepEmulation();

        publishOdometry();
        publishPose();
        publishAccumulatedOdometry();
        publishGoalLanding();
        publishLocalized();

        refreshRate.sleep();
    }

    pingTimer_.stop();
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
    tapInternalGoalSolution_.connectTap();
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
    tapInternalGoalSolution_.disconnectTap();
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
void Motion::executeSolution(Solution<GridSolutionItem> solution)
{
    ROS_DEBUG_STREAM_NAMED("motion", "Communicated solution: " << endl << solution);

    motionController_.execute(solution);
    positionEstimator_.resetAccumulatedOdometry();

    simulatedT_ = 0.0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Motion::onConfigChange(MotionConfig& config, uint32_t level)
{
    positionEstimator_.setConfiguration(config);
    motionController_.setConfiguration(config);

    cv::Mat R = FactoryApsNoise::fromConfiguration(config);
    tapAps_.getSensor()->setR(R);
    tapAps_.getSensor()->enable(config.sensor_aps_enabled);

    R = FactoryOdometryNoise::fromConfiguration(config);
    tapOdometry_.getSensor()->setR(R);
    tapOdometry_.getSensor()->enable(config.sensor_odometry_enabled);

    isCustomActionEnabled_ = config.custom_action_enabled;

    ROS_INFO_STREAM_NAMED("motion",
        "Custom action enabled [t/f]: " <<
        config.custom_action_enabled);

    ROS_INFO_STREAM_NAMED("motion",
        "Emergency Controller: linear velocity gain []: " <<
        config.emergency_controller_kv);
    ROS_INFO_STREAM_NAMED("motion",
        "Emergency Controller: angular velocity gain []: " <<
        config.emergency_controller_kw);
    ROS_INFO_STREAM_NAMED("motion",
        "Emergency Controller: maximum angular velocity in emergency mode [rad/s]: " <<
        config.emergency_controller_max_angular_velocity);
    ROS_INFO_STREAM_NAMED("motion",
        "Emergency Controller: maximum linear velocity in emergency mode [m/s]: " <<
        config.emergency_controller_max_linear_velocity);

    ROS_INFO_STREAM_NAMED("motion",
        "Manual Controller: linear velocity gain []: " <<
        config.manual_controller_kv);
    ROS_INFO_STREAM_NAMED("motion",
        "Manual Controller: angular velocity gain []: " <<
        config.manual_controller_kw);
    ROS_INFO_STREAM_NAMED("motion",
        "Manual Controller: maximum angular velocity in manual mode [rad/s]: " <<
        config.manual_controller_max_angular_velocity);
    ROS_INFO_STREAM_NAMED("motion",
        "Manual Controller: maximum linear velocity in manual mode [m/s]: " <<
        config.manual_controller_max_linear_velocity);

    ROS_INFO_STREAM_NAMED("motion",
        "Maximum physical angular acceleration [rad/s^2]: " <<
        config.physical_max_angular_acceleration);
    ROS_INFO_STREAM_NAMED("motion",
        "Maximum physical angular velocity [rad/s]: " <<
        config.physical_max_angular_velocity);
    ROS_INFO_STREAM_NAMED("motion",
        "Maximum physical linear acceleration [m/s^2]: " <<
        config.physical_max_linear_acceleration);
    ROS_INFO_STREAM_NAMED("motion",
        "Maximum physical linear velocity [m/s]: " <<
        config.physical_max_linear_velocity);
    ROS_INFO_STREAM_NAMED("motion",
        "Minimum physical angular velocity [rad/s]: " <<
        config.physical_min_angular_velocity);
    ROS_INFO_STREAM_NAMED("motion",
        "Minimum physical linear velocity [m/s]: " <<
        config.physical_min_linear_velocity);

    ROS_INFO_STREAM_NAMED("motion",
        "Path-follow Controller: adaptive look-ahead enabled [t/f]: " <<
        config.pathfollow_controller_adaptive_lookahead_enabled);
    ROS_INFO_STREAM_NAMED("motion",
        "Path-follow Controller: distance used to check if the goal was reached [m]: " <<
        config.pathfollow_controller_goal_reached_distance);
    ROS_INFO_STREAM_NAMED("motion",
        "Path-follow Controller: linear velocity gain []: " <<
        config.pathfollow_controller_kv);
    ROS_INFO_STREAM_NAMED("motion",
        "Path-follow Controller: angular velocity gain []: " <<
        config.pathfollow_controller_kw);
    ROS_INFO_STREAM_NAMED("motion",
        "Path-follow Controller: depth of the landing zone [m]: " <<
        config.pathfollow_controller_landing_depth);
    ROS_INFO_STREAM_NAMED("motion",
        "Path-follow Controller: width of the landing zone [m]: " <<
        config.pathfollow_controller_landing_width);
    ROS_INFO_STREAM_NAMED("motion",
        "Path-follow Controller: linear acceleration [m/s^2]: " <<
        config.pathfollow_controller_linear_acceleration);
    ROS_INFO_STREAM_NAMED("motion",
        "Path-follow Controller: maximum angular velocity [rad/s]: " <<
        config.pathfollow_controller_max_angular_velocity);
    ROS_INFO_STREAM_NAMED("motion",
        "Path-follow Controller: maximum linear velocity [m/s]: " <<
        config.pathfollow_controller_max_linear_velocity);
    ROS_INFO_STREAM_NAMED("motion",
        "Path-follow Controller: maximum look-ahead distance [m]: " <<
        config.pathfollow_controller_max_look_ahead_distance);
    ROS_INFO_STREAM_NAMED("motion",
        "Path-follow Controller: minimum linear velocity allowed [m/s]: " <<
        config.pathfollow_controller_min_linear_velocity);
    ROS_INFO_STREAM_NAMED("motion",
        "Path-follow Controller: minimum look-ahead distance [m]: " <<
        config.pathfollow_controller_min_look_ahead_distance);
    ROS_INFO_STREAM_NAMED("motion",
        "Path-follow Controller: straight distance that is considered small [m]: " <<
        config.pathfollow_controller_small_straight_distance);
    ROS_INFO_STREAM_NAMED("motion",
        "Path-follow Controller: travel linear velocity during turns [m/s]: " <<
        config.pathfollow_controller_turning_linear_velocity);
    ROS_INFO_STREAM_NAMED("motion",
        "Path-follow Controller: travel reduced velocity radius during turns [m]: " <<
        config.pathfollow_controller_turning_zone_radius);
    ROS_INFO_STREAM_NAMED("motion",
        "Path-follow Controller: zero-point look-ahead distance [m]: " <<
        config.pathfollow_controller_zero_look_ahead_distance);

    ROS_INFO_STREAM_NAMED("motion",
        "Rotation Controller: angle used to check if the goal was reached [rad]: " <<
        config.rotation_controller_goal_reached_angle);
    ROS_INFO_STREAM_NAMED("motion",
        "Rotation Controller: derivative constant []: " <<
        config.rotation_controller_kd);
    ROS_INFO_STREAM_NAMED("motion",
        "Rotation Controller: integral constant []: " <<
        config.rotation_controller_ki);
    ROS_INFO_STREAM_NAMED("motion",
        "Rotation Controller: proportional constant []: " <<
        config.rotation_controller_kp);
    ROS_INFO_STREAM_NAMED("motion",
        "Rotation Controller: linear velocity gain []: " <<
        config.rotation_controller_kv);
    ROS_INFO_STREAM_NAMED("motion",
        "Rotation Controller: angular velocity gain []: " <<
        config.rotation_controller_kw);
    ROS_INFO_STREAM_NAMED("motion",
        "Rotation Controller: minimum angular velocity during motion [rad/s]: " <<
        config.rotation_controller_min_angular_velocity);
    ROS_INFO_STREAM_NAMED("motion",
        "Rotation Controller: rotation velocity [rad/s]: " <<
        config.rotation_controller_rotation_velocity);

    ROS_INFO_STREAM_NAMED("motion",
        "APS enabled: " <<
        config.sensor_aps_enabled);
    ROS_INFO_STREAM_NAMED("motion",
        "ODOMETRY enabled: " <<
        config.sensor_odometry_enabled);

    ROS_INFO_STREAM_NAMED("motion",
        "Stop Controller: minimum linear velocity allowed [m/s]: " <<
        config.stop_controller_min_linear_velocity);
    ROS_INFO_STREAM_NAMED("motion",
        "Stop Controller: normal deceleration [m/s^2]: " <<
        config.stop_controller_normal_linear_deceleration);

    ROS_INFO_STREAM_NAMED("motion",
        "Heading component of the APS noise [rad]: " <<
        config.ukf_aps_error_heading);
    ROS_INFO_STREAM_NAMED("motion",
        "Location (X and Y) component of the APS noise [m]: " <<
        config.ukf_aps_error_location);
    ROS_INFO_STREAM_NAMED("motion",
        "Angular velocity component of the odometry noise [rad/s]: " <<
        config.ukf_odometry_error_angular);
    ROS_INFO_STREAM_NAMED("motion",
        "Linear velocity component of the odometry noise [m/s]: " <<
        config.ukf_odometry_error_linear);
    ROS_INFO_STREAM_NAMED("motion",
        "Heading component of the robot noise [rad]: " <<
        config.ukf_robot_error_heading);
    ROS_INFO_STREAM_NAMED("motion",
        "Location (X and Y) component of the robot noise [m]: " <<
        config.ukf_robot_error_location);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Motion::performCustomAction()
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Motion::pingCallback(const ros::TimerEvent& event)
{
    std_msgs::Bool message;
    message.data = true;

    pubPing_.publish(message);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Motion::publishAccumulatedOdometry()
{
    srslib_framework::MsgPose message = PoseMessageFactory::pose2Msg(
        positionEstimator_.getAccumulatedOdometry());
    pubRobotAccOdometry_.publish(message);
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
            ROS_DEBUG_STREAM_NAMED("motion", "Arrived at goal: " <<
                motionController_.getFinalGoal());
        }
        else
        {
            messageGoalArrived.data = false;
            ROS_DEBUG_STREAM_NAMED("motion", "Departed for goal: " <<
                motionController_.getFinalGoal());
        }

        pubStatusGoalArrived_.publish(messageGoalArrived);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Motion::publishGoalLanding()
{
    geometry_msgs::PolygonStamped messageLanding;

    messageLanding.header.frame_id = "map";
    messageLanding.header.stamp = ros::Time::now();

    vector<geometry_msgs::Point32> polygon;
    vector<Pose<>> landingArea;

    motionController_.getLanding(landingArea);
    for (auto pose : landingArea)
    {
        geometry_msgs::Point32 corner;
        corner.x = pose.x;
        corner.y = pose.y;
        corner.z = 0.0;

        polygon.push_back(corner);
    }
    messageLanding.polygon.points = polygon;

    pubStatusGoalLanding_.publish(messageLanding);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Motion::publishOdometry()
{
    Pose<> currentPose = positionEstimator_.getPose();

    if (currentPose.isValid())
    {
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
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Motion::publishLocalized()
{
    std_msgs::Bool messageRobotLocalized;
    messageRobotLocalized.data = positionEstimator_.isPoseValid();
    pubRobotLocalized_.publish(messageRobotLocalized);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Motion::publishPose()
{
    Pose<> robotPose = positionEstimator_.getPose();

    if (robotPose.isValid())
    {
        srslib_framework::MsgPose message = PoseMessageFactory::pose2Msg(robotPose);
        pubRobotPose_.publish(message);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Motion::reset(Pose<> pose0)
{
    positionEstimator_.reset(pose0);
    motionController_.reset();
    simulatedT_ = 0.0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Motion::scanTapsForData()
{
    // If APS has reported a position and it is the first time that Motion
    // gets it, make sure that the Position Estimator knows of it.
    if (!tapAps_.hasNeverReported() && !positionEstimator_.isPoseValid())
    {
        ROS_DEBUG_STREAM("Reporting its first position: " << tapAps_.getPose());

        reset(tapAps_.getPose());
    }

    if (tapInitialPose_.newDataAvailable())
    {
        ROS_DEBUG_STREAM("Received initial pose: " << tapInitialPose_.getPose());

        reset(tapInitialPose_.getPose());
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

        if (tapJoyAdapter_.getCustomActionState())
        {
            performCustomAction();
        }

        // Store the latch state for later use
        isJoystickLatched_ = tapJoyAdapter_.getLatchState();
    }

    // If there is a new solution has been communicated
    if (tapInternalGoalSolution_.newDataAvailable())
    {
        if (!isJoystickLatched_)
        {
            executeSolution(tapInternalGoalSolution_.getSolution());
        }
    }

    // Check if odometry or APS data is available
    isOdometryAvailable_ = tapOdometry_.newDataAvailable();
    isApsAvailable_ = tapAps_.newDataAvailable();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Motion::stepEmulation()
{
    // If the brain stem is disconnected, emulate odometry and initial position
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

        if (!positionEstimator_.isPoseValid())
        {
            // Establish an "acceptable" initial position, at least until the
            // Position Estimator can get its bearings
            tapInitialPose_.set(Pose<>(TimeMath::time2number(currentTime_), 3.0, 3.0, 0));
            reset(tapInitialPose_.getPose());
        }
    }
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

    // TODO: Move the odometry to be a sensor of the UKF and not a command
    Odometry<> currentOdometry = tapOdometry_.getOdometry();

    // Provide the command to the position estimator if available
    if (isOdometryAvailable_ || isApsAvailable_)
    {
        positionEstimator_.run(isOdometryAvailable_ ? &currentOdometry : nullptr);
    }

    // Run the motion controller if the estimated position is valid. No
    // motion is allowed if a position has been established
    Pose<> currentPose = positionEstimator_.getPose();
    if (currentPose.isValid())
    {
        motionController_.run(currentPose, currentOdometry, currentJoystickCommand);
    }
}

} // namespace srs
