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
#include <std_msgs/Float64MultiArray.h>
#include <tf/transform_broadcaster.h>

#include <srsnode_motion/MotionConfig.h>
#include <srslib_framework/Pose.h>

#include <srslib_framework/math/AngleMath.hpp>
#include <srslib_framework/planning/pathplanning/Solution.hpp>
#include <srslib_framework/robotics/robot_profile/ChuckProfile.hpp>
#include <srslib_framework/robotics/Odometry.hpp>
#include <srslib_framework/ros/message/PoseMessageFactory.hpp>
#include <srslib_framework/ros/topics/ChuckTopics.hpp>

#include <srsnode_motion/tap/sensor_frame/aps/FactoryApsNoise.hpp>
#include <srsnode_motion/tap/sensor_frame/imu/FactoryImuNoise.hpp>
#include <srsnode_motion/tap/sensor_frame/odometry/FactoryOdometryNoise.hpp>
#include <srsnode_motion/FactoryRobotNoise.hpp>
#include <srsnode_motion/FactoryRobotProfile.hpp>

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
    simulatedT_(0.0),
    pubRobotPose_(ChuckTopics::internal::ROBOT_POSE),
    pubRobotAccOdometry_(ChuckTopics::debug::ACC_ODOMETRY)
{
    positionEstimator_.addSensor(tapAps_.getSensor());
    positionEstimator_.addSensor(tapSensorFrame_.getSensorImu());

    configServer_.setCallback(boost::bind(&Motion::onConfigChange, this, _1, _2));

    pubOdometry_ = rosNodeHandle_.advertise<nav_msgs::Odometry>(
        "/internal/state/robot/twist", 50);

    pubPing_ = rosNodeHandle_.advertise<std_msgs::Bool>(
        "/internal/state/ping", 1);

    pubImu_ = rosNodeHandle_.advertise<srslib_framework::Imu>(
        "/internal/sensors/imu/calibrated", 100);

    pubRobotLocalized_ = rosNodeHandle_.advertise<std_msgs::Bool>(
        "/internal/state/robot/localized", 1, true);

    pubStatusGoalArrived_ = rosNodeHandle_.advertise<std_msgs::Bool>(
        "/internal/state/goal/arrived", 1);
    pubStatusGoalLanding_ = rosNodeHandle_.advertise<geometry_msgs::PolygonStamped>(
        "/internal/state/goal/landing", 1);

    pubCovariance_ = rosNodeHandle_.advertise<std_msgs::Float64MultiArray>(
        "/internal/state/robot/covariance", 1);

    rosNodeHandle_.param("emulation", isEmulationEnabled_, false);
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

        stepEmulation();

        evaluateTriggers();

        scanTapsForData();
        publishArrived();

        stepNode();

        publishImu();
        publishOdometry();
        publishPose();
        publishGoalLanding();
        publishLocalized();
        publishCovariance();

        pubRobotAccOdometry_.publish(positionEstimator_.getAccumulatedOdometry());

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
    tapSensorFrame_.connectTap();
    tapAps_.connectTap();
    tapInternalGoalSolution_.connectTap();
    tapMap_.connectTap();

    triggerStop_.connectService();
    triggerShutdown_.connectService();
    triggerPause_.connectService();
    triggerExecuteSolution_.connectService();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Motion::disconnectAllTaps()
{
    tapBrainStem_.disconnectTap();
    tapSensorFrame_.disconnectTap();
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

    if (triggerPause_.newRequestPending())
    {
        // Depending on the state of the pause button, enable or disable
        // the autonomous mode
        if (triggerPause_.getRequest())
        {
            motionController_.switchToManual();
        }
        else
        {
            motionController_.switchToAutonomous();
        }
    }

    // TODO: Convert this into a ROS action
    if (triggerExecuteSolution_.newRequestPending())
    {
        if (!isJoystickLatched_)
        {
            executeSolution(triggerExecuteSolution_.getRequest());
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Motion::executeSolution(Solution<GridSolutionItem> solution)
{
    ROS_DEBUG_STREAM_NAMED("motion", "Communicated solution: " << endl << solution);

    motionController_.execute(solution);
    positionEstimator_.resetAccumulatedOdometry(nullptr);

    simulatedT_ = 0.0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Motion::onConfigChange(MotionConfig& config, uint32_t level)
{
    cv::Mat apsR = FactoryApsNoise<double>::fromConfiguration(config);
    tapAps_.getSensor()->setR(apsR);
    tapAps_.getSensor()->enable(config.sensor_aps_enabled);

    cv::Mat odometryR = FactoryOdometryNoise<double>::fromConfiguration(config);
    tapSensorFrame_.getSensorOdometry()->setR(odometryR);
    tapSensorFrame_.getSensorOdometry()->enable(config.sensor_odometry_enabled);

    cv::Mat robotQ = FactoryRobotNoise<double>::fromConfiguration(config);
    positionEstimator_.setRobotQ(robotQ);
    positionEstimator_.setP0Value(config.ukf_robot_p0);
    positionEstimator_.enableNaive(config.naive_sensor_fusion_enabled);

    RobotProfile robotProfile = FactoryRobotProfile::fromConfiguration(config);
    motionController_.setRobotProfile(robotProfile);

    isCustomActionEnabled_ = config.custom_action_enabled;

    ROS_INFO_STREAM_NAMED("motion",
        "Custom action enabled [t/f]: " <<
        config.custom_action_enabled);

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
        "Minimum absolute physical distance to goal [m]: " <<
        config.physical_min_distance_to_goal);
    ROS_INFO_STREAM_NAMED("motion",
        "Minimum physical linear velocity [m/s]: " <<
        config.physical_min_linear_velocity);

    ROS_INFO_STREAM_NAMED("motion",
        "APS enabled: " <<
        config.sensor_aps_enabled);
    ROS_INFO_STREAM_NAMED("motion",
        "IMU enabled: " <<
        config.sensor_imu_enabled);
    ROS_INFO_STREAM_NAMED("motion",
        "ODOMETRY enabled: " <<
        config.sensor_odometry_enabled);

    ROS_INFO_STREAM_NAMED("motion",
        "Single command mode [t/f]: " <<
        config.single_command_mode);

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
        "Yaw component of the IMU noise [rad]: " <<
        config.ukf_imu_error_yaw);

    ROS_INFO_STREAM_NAMED("motion",
        "Heading component of the robot noise [rad]: " <<
        config.ukf_robot_error_heading);
    ROS_INFO_STREAM_NAMED("motion",
        "X component of the robot noise [m]: " <<
        config.ukf_robot_error_location_x);
    ROS_INFO_STREAM_NAMED("motion",
        "Y component of the robot noise [m]: " <<
        config.ukf_robot_error_location_y);
    ROS_INFO_STREAM_NAMED("motion",
        "Linear velocity component of the robot noise [m/s]: " <<
        config.ukf_robot_error_linear);
    ROS_INFO_STREAM_NAMED("motion",
        "Angular velocity component of the robot noise [rad/s]: " <<
        config.ukf_robot_error_angular);

    ROS_INFO_STREAM_NAMED("motion",
        "Common value for the initialization of P0: " <<
        config.ukf_robot_p0);

    ROS_INFO_STREAM_NAMED("motion",
        "Emergency Controller: linear velocity gain []: " <<
        config.controller_emergency_kv);
    ROS_INFO_STREAM_NAMED("motion",
        "Emergency Controller: angular velocity gain []: " <<
        config.controller_emergency_kw);
    ROS_INFO_STREAM_NAMED("motion",
        "Emergency Controller: maximum angular velocity in emergency mode [rad/s]: " <<
        config.controller_emergency_max_angular_velocity);
    ROS_INFO_STREAM_NAMED("motion",
        "Emergency Controller: maximum linear velocity in emergency mode [m/s]: " <<
        config.controller_emergency_max_linear_velocity);

    ROS_INFO_STREAM_NAMED("motion",
        "Manual Controller: linear velocity gain []: " <<
        config.controller_manual_kv);
    ROS_INFO_STREAM_NAMED("motion",
        "Manual Controller: angular velocity gain []: " <<
        config.controller_manual_kw);
    ROS_INFO_STREAM_NAMED("motion",
        "Manual Controller: maximum angular velocity in manual mode [rad/s]: " <<
        config.controller_manual_max_angular_velocity);
    ROS_INFO_STREAM_NAMED("motion",
        "Manual Controller: maximum linear velocity in manual mode [m/s]: " <<
        config.controller_manual_max_linear_velocity);

    ROS_INFO_STREAM_NAMED("motion",
        "Path-follow Controller: adaptive look-ahead enabled [t/f]: " <<
        config.controller_pathfollow_adaptive_lookahead_enabled);
    ROS_INFO_STREAM_NAMED("motion",
        "Path-follow Controller: distance used to check if the goal was reached [m]: " <<
        config.controller_pathfollow_goal_reached_distance);
    ROS_INFO_STREAM_NAMED("motion",
        "Path-follow Controller: linear velocity gain []: " <<
        config.controller_pathfollow_kv);
    ROS_INFO_STREAM_NAMED("motion",
        "Path-follow Controller: angular velocity gain []: " <<
        config.controller_pathfollow_kw);
    ROS_INFO_STREAM_NAMED("motion",
        "Path-follow Controller: depth of the landing zone [m]: " <<
        config.controller_pathfollow_landing_depth);
    ROS_INFO_STREAM_NAMED("motion",
        "Path-follow Controller: width of the landing zone [m]: " <<
        config.controller_pathfollow_landing_width);
    ROS_INFO_STREAM_NAMED("motion",
        "Path-follow Controller: linear acceleration [m/s^2]: " <<
        config.controller_pathfollow_linear_acceleration);
    ROS_INFO_STREAM_NAMED("motion",
        "Path-follow Controller: maximum angular velocity [rad/s]: " <<
        config.controller_pathfollow_max_angular_velocity);
    ROS_INFO_STREAM_NAMED("motion",
        "Path-follow Controller: maximum linear velocity [m/s]: " <<
        config.controller_pathfollow_max_linear_velocity);
    ROS_INFO_STREAM_NAMED("motion",
        "Path-follow Controller: maximum look-ahead distance [m]: " <<
        config.controller_pathfollow_max_look_ahead_distance);
    ROS_INFO_STREAM_NAMED("motion",
        "Path-follow Controller: minimum linear velocity allowed [m/s]: " <<
        config.controller_pathfollow_min_linear_velocity);
    ROS_INFO_STREAM_NAMED("motion",
        "Path-follow Controller: minimum look-ahead distance [m]: " <<
        config.controller_pathfollow_min_look_ahead_distance);
    ROS_INFO_STREAM_NAMED("motion",
        "Path-follow Controller: straight distance that is considered small [m]: " <<
        config.controller_pathfollow_small_straight_distance);
    ROS_INFO_STREAM_NAMED("motion",
        "Path-follow Controller: travel linear velocity during turns [m/s]: " <<
        config.controller_pathfollow_turning_linear_velocity);
    ROS_INFO_STREAM_NAMED("motion",
        "Path-follow Controller: travel reduced velocity radius during turns [m]: " <<
        config.controller_pathfollow_turning_zone_radius);
    ROS_INFO_STREAM_NAMED("motion",
        "Path-follow Controller: zero-point look-ahead distance [m]: " <<
        config.controller_pathfollow_zero_look_ahead_distance);

    ROS_INFO_STREAM_NAMED("motion",
        "Rotation Controller: angle used to check if the goal was reached [rad]: " <<
        config.controller_rotation_goal_reached_angle);
    ROS_INFO_STREAM_NAMED("motion",
        "Rotation Controller: derivative constant []: " <<
        config.controller_rotation_kd);
    ROS_INFO_STREAM_NAMED("motion",
        "Rotation Controller: integral constant []: " <<
        config.controller_rotation_ki);
    ROS_INFO_STREAM_NAMED("motion",
        "Rotation Controller: proportional constant []: " <<
        config.controller_rotation_kp);
    ROS_INFO_STREAM_NAMED("motion",
        "Rotation Controller: linear velocity gain []: " <<
        config.controller_rotation_kv);
    ROS_INFO_STREAM_NAMED("motion",
        "Rotation Controller: angular velocity gain []: " <<
        config.controller_rotation_kw);
    ROS_INFO_STREAM_NAMED("motion",
        "Rotation Controller: minimum angular velocity during motion [rad/s]: " <<
        config.controller_rotation_min_angular_velocity);
    ROS_INFO_STREAM_NAMED("motion",
        "Rotation Controller: rotation velocity [rad/s]: " <<
        config.controller_rotation_rotation_velocity);

    ROS_INFO_STREAM_NAMED("motion",
        "Stop Controller: minimum linear velocity allowed [m/s]: " <<
        config.controller_stop_min_linear_velocity);
    ROS_INFO_STREAM_NAMED("motion",
        "Stop Controller: normal deceleration [m/s^2]: " <<
        config.controller_stop_normal_linear_deceleration);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Motion::pingCallback(const ros::TimerEvent& event)
{
    std_msgs::Bool message;
    message.data = true;

    pubPing_.publish(message);

    double delay = (event.current_real - event.current_expected).toSec();

    // We should never be falling behind by more than 500ms
    if ( delay > (1.0f / PING_HZ) * MAX_ALLOWED_PING_DELAY)
    {
        ROS_ERROR_STREAM_NAMED("motion", "Motion ping exceeded allowable delay: " << delay );
    }
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
void Motion::publishCovariance()
{
    cv::Mat covariance = positionEstimator_.getCovariance();

    std_msgs::Float64MultiArray messageCovariance;
    messageCovariance.data.clear();

    for (int r = 0; r < covariance.rows; r++)
    {
        for (int c = 0; c < covariance.cols; c++)
        {
            messageCovariance.data.push_back(covariance.at<double>(r, c));
        }
    }

    pubCovariance_.publish(messageCovariance);
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
void Motion::publishImu()
{
    srslib_framework::Imu message = ImuMessageFactory::imu2Msg(tapSensorFrame_.getCalibratedImu());
    pubImu_.publish(message);
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
        pubRobotPose_.publish(robotPose);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Motion::reset(Pose<> pose0)
{
    positionEstimator_.reset(pose0);
    motionController_.reset();

    simulatedT_ = 0.0;

    // Make sure that the calibrated IMU data
    // knows about the new pose
    tapSensorFrame_.setTrueYaw(pose0.theta);
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
        ROS_DEBUG_STREAM("Received initial pose: " << tapInitialPose_.peek());

        reset(tapInitialPose_.pop());
    }

    // If the joystick was touched, read the state of the latch
    if (tapJoyAdapter_.newDataAvailable())
    {
        // If the controller emergency button has been pressed, tell
        // the motion controller immediately
        if (tapJoyAdapter_.getButtonEmergency())
        {
            motionController_.emergencyStop();
        }

        if (tapJoyAdapter_.getButtonAction())
        {
            Pose<> pose = positionEstimator_.getPose();
            positionEstimator_.resetAccumulatedOdometry(&pose);
        }

        // Store the latch state for later use
        isJoystickLatched_ = tapJoyAdapter_.getLatchState();
    }

    // Check if odometry or APS data is available
    isOdometryAvailable_ = tapSensorFrame_.newOdometryDataAvailable();
    isApsAvailable_ = tapAps_.newDataAvailable();
    isImuAvailable_ = tapSensorFrame_.newImuCalibratedDataAvailable();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Motion::stepEmulation()
{
    // If emulation is enabled, feed velocity commands back to the
    // odometry sensor, and initialize the first pose
    if (isEmulationEnabled_)
    {
        ROS_WARN_STREAM_ONCE_NAMED("motion", "Emulation enabled");

        Velocity<> currentCommand = motionController_.getExecutingCommand();
        Pose<> currentPose = positionEstimator_.getPose();

        simulatedT_ += 1.0 / REFRESH_RATE_HZ;
        tapSensorFrame_.setOdometry(Odometry<>(Velocity<>(
            simulatedT_,
            currentCommand.linear,
            currentCommand.angular)));
        tapSensorFrame_.setRawImu(Imu<>(
            simulatedT_,
            currentPose.theta, 0.0, 0.0,
            currentCommand.angular, 0.0, 0.0));

        if (!positionEstimator_.isPoseValid())
        {
            // Establish an "acceptable" initial position, at least until the
            // Position Estimator can get its bearings
            tapInitialPose_.set(Pose<>(TimeMath::time2number(currentTime_), 3.0, 3.0, 0));
            reset(tapInitialPose_.pop());
        }
    }
    else
    {
        ROS_WARN_STREAM_ONCE_NAMED("motion", "Emulation disabled");
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
        // If the joystick has been latched or the current robot pose
        // is not valid switch to manual mode
        if (isJoystickLatched_ || !positionEstimator_.isPoseValid())
        {
            motionController_.switchToManual();
        }
        else
        {
            motionController_.switchToAutonomous();
        }
    }

    // TODO: Move the odometry to be a sensor of the UKF and not a command
    Odometry<> currentOdometry = tapSensorFrame_.getOdometry();

    // Perform a number of actions if the robot is not moving
    if (VelocityMath::equal(currentOdometry.velocity, Velocity<>::ZERO))
    {
        // If an absolute position is available, tell to the sensor frame tap
        // about it. The tap will use the accumulated yaw values to calculate
        // an average of the difference between the theta of the absolute position
        // and the absolute yaw returned by the IMU
        if (isApsAvailable_)
        {
            // Tell the tap to start accumulating yaw values. This has no
            // effect if the tap had already started to accumulate
            // ###FS tapSensorFrame_.startAccumulatingYaw();
            tapSensorFrame_.setTrueYaw(tapAps_.getPose().theta);
            // ###FS tapSensorFrame_.addTrueYaw(tapAps_.getPose().theta);
        }
    }
    else
    {
        // ###FS tapSensorFrame_.stopAccumulatingYaw();
    }

    // Provide the command to the position estimator if available
    if (isOdometryAvailable_ || isApsAvailable_ || isImuAvailable_)
    {
        Imu<> currentImu = tapSensorFrame_.getRawImu(); // ###FS tapSensorFrame_.getCalibratedImu();
        Pose<> currentAps = tapAps_.getPose();

        positionEstimator_.run(
            isOdometryAvailable_ ? &currentOdometry : nullptr,
            isImuAvailable_ ? &currentImu : nullptr,
            isApsAvailable_ ? &currentAps : nullptr
        );
    }

    // Run the motion controller if the estimated position is valid. No
    // motion is allowed if a position has been established
    Pose<> currentPose = positionEstimator_.getPose();
    motionController_.run(currentPose, currentOdometry, currentJoystickCommand);
}

} // namespace srs
