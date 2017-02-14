/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#include <srsnode_odometry/OdometryPositionEstimator.hpp>

#include <srslib_framework/math/TimeMath.hpp>
#include <srslib_framework/math/PoseMath.hpp>
#include <srslib_framework/ros/message/PoseMessageFactory.hpp>
#include <srslib_framework/chuck/ChuckTopics.hpp>
#include <srslib_framework/chuck/ChuckTransforms.hpp>

namespace srs {

OdometryPositionEstimator::OdometryPositionEstimator(std::string nodeName) :
	nodeHandle_(nodeName),
	lastPoseTime_(ros::Time::now() + ros::Duration(5.0)),
	pose_(0.0, 0.0, 0.0),
	pingTimer_(),
	broadcaster_(),
	resetPoseSub_(),
	rawVelocityCmdSub_(),
	rawOdometryRpmSub_(),
	rpmVelocityCmdPub_(),
	odometryPosePub_(),
	odometryPoseEstimatePub_(),
	pingPub_(),
	wheelbaseLength_(0.5235),
	leftWheelRadius_(0.10243),
	rightWheelRadius_(0.10243)
{

}

OdometryPositionEstimator::~OdometryPositionEstimator()
{
}

void OdometryPositionEstimator::run()
{
	ros::Rate refreshRate(REFRESH_RATE_HZ);

	readParams();
	connect();

	char* pszLeftWheelRadius = getenv("ROBOT_LEFT_WHEEL_RADIUS");
	char* pszRightWheelRadius = getenv("ROBOT_RIGHT_WHEEL_RADIUS");
	char* pszWheelBaseLength = getenv("ROBOT_WHEEL_BASE");

	if( pszLeftWheelRadius )
	{
		sscanf( pszLeftWheelRadius, "%lf", &leftWheelRadius_ );
	}

	if( pszRightWheelRadius )
	{
		sscanf( pszRightWheelRadius, "%lf", &rightWheelRadius_ );
	}

	if( pszWheelBaseLength )
	{
		sscanf( pszWheelBaseLength, "%lf", &wheelbaseLength_ );
	}

    ROS_INFO_STREAM("Odometry Configuration:"
		" wheelbaseLength_=" << wheelbaseLength_ <<
		", leftWheelRadius_=" << leftWheelRadius_ <<
		", rightWheelRadius_=" << rightWheelRadius_);

	while(ros::ok())
	{
		ros::spinOnce();

		refreshRate.sleep();
	}

	disconnect();
}

void OdometryPositionEstimator::readParams()
{
    nodeHandle_.param("linear_acceleration_rate", linear_acceleration_rate_, linear_acceleration_rate_);
    nodeHandle_.param("angular_acceleration_rate", angular_acceleration_rate_, angular_acceleration_rate_);
    nodeHandle_.param("velocity_loop_delays", velocity_loop_delays_, velocity_loop_delays_);
}

void OdometryPositionEstimator::connect()
{
	// Subscriber to get odometry reading (in rpm) and calculates the robot pose
	rawOdometryRpmSub_ = nodeHandle_.subscribe<srslib_framework::OdometryRpm>(ChuckTopics::driver::BRAINSTEM_ODOMETRY_RPM, 10,
		std::bind( &OdometryPositionEstimator::CalculateRobotPose, this, std::placeholders::_1 ));

	// Subscriber to reset robot pose when necessary
	resetPoseSub_ = nodeHandle_.subscribe<geometry_msgs::PoseWithCovarianceStamped>(ChuckTopics::internal::ODOMETRY_INITIAL_POSE, 1,
			std::bind( &OdometryPositionEstimator::ResetOdomPose, this, std::placeholders::_1 ));

	// Subscriber to transform velocity command from linear/angular velocity to RPM format
	rawVelocityCmdSub_ = nodeHandle_.subscribe<geometry_msgs::Twist>(ChuckTopics::driver::ODOMETRY_CMD_VELOCITY, 10,
			std::bind( &OdometryPositionEstimator::TransformVeclocityToRPM, this, std::placeholders::_1 ));

	odometryPosePub_ = nodeHandle_.advertise<nav_msgs::Odometry>(ChuckTopics::sensor::ODOMETRY_POSE, 10);

    odometryPoseEstimatePub_ = nodeHandle_.advertise<nav_msgs::Odometry>(ChuckTopics::internal::ODOMETRY_VELOCITY_ESTIMATE, 10);

    rpmVelocityCmdPub_ = nodeHandle_.advertise<srslib_framework::OdometryRpm>(ChuckTopics::driver::BRAINSTEM_ODOMETRY_RPM_CMD, 10);

	pingPub_ = nodeHandle_.advertise<std_msgs::Bool>(ChuckTopics::driver::BRAINSTEM_CMD_PING, 1);

	// Register dynamic configuration callback
	configServer_.setCallback(boost::bind(&OdometryPositionEstimator::cfgCallback, this, _1, _2));

    // Start the ping timer
    pingTimer_ = nodeHandle_.createTimer(ros::Duration(1.0 / PING_HZ),
        boost::bind(&OdometryPositionEstimator::pingCallback, this, _1));
}

void OdometryPositionEstimator::disconnect()
{
	odometryPosePub_.shutdown();
	rpmVelocityCmdPub_.shutdown();
}

void OdometryPositionEstimator::CalculateRobotPose( const srslib_framework::OdometryRpm::ConstPtr& wheelRPM )
{
	// If no initial pose is provided, return immediately without any calculation
	if(pose_.x == (-1.0) && pose_.y == (-1.0) && pose_.theta == (-1.0))
	{
		ROS_ERROR_ONCE("No initial pose provided");
		return;
	}

	static ros::Time s_lastTime = wheelRPM->header.stamp;
	static Pose<> s_lastPose = pose_;

	// Update current time and calculate time interval between two consecutive messages
	ros::Time currentTime = wheelRPM->header.stamp;

	double dfTimeDelta = (currentTime - s_lastTime).toSec();

	// Calculate linear and angular velocity
	double linearVelocity = 0.0;
	double angularVelocity = 0.0;
	GetRawOdometryVelocity(wheelRPM->left_wheel_rpm, wheelRPM->right_wheel_rpm, linearVelocity, angularVelocity);

	// Message declarations
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.frame_id = ChuckTransforms::ODOMETRY;
	odom_trans.child_frame_id = ChuckTransforms::BASE_FOOTPRINT;

    constexpr static double ANGULAR_VELOCITY_EPSILON = 0.000001; // [rad/s] (0.0573 [deg/s])

    // Check for the special case in which omega is 0 (the robot is moving straight)
	if (abs(angularVelocity) > ANGULAR_VELOCITY_EPSILON)
    {
        double r = linearVelocity / angularVelocity;

        pose_.x = pose_.x + r * sin(pose_.theta + angularVelocity * dfTimeDelta) - r * sin(pose_.theta),
        pose_.y = pose_.y + r * cos(pose_.theta) - r * cos(pose_.theta + angularVelocity * dfTimeDelta),
		pose_.theta = AngleMath::normalizeRad<>(pose_.theta + angularVelocity * dfTimeDelta);
    }
    else
    {
    	pose_ = PoseMath::translate<>(pose_, linearVelocity * dfTimeDelta, 0.0);
    }

	// Display status in DEBUG MODE whenever pose has been changed
	if(pose_.x != s_lastPose.x || pose_.y != s_lastPose.y || pose_.theta != s_lastPose.theta)
	{
		ROS_DEBUG( "Estimated Pose Changed: x=%f, y=%f, theta=%f",
					pose_.x, pose_.y, pose_.theta );
	}

	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw( pose_.theta );

	// Publish the TF
	odom_trans.header.stamp = currentTime;
	odom_trans.transform.translation.x = pose_.x;
	odom_trans.transform.translation.y = pose_.y;
	odom_trans.transform.translation.z = 0.0;
	odom_trans.transform.rotation = odom_quat;

	broadcaster_.sendTransform( odom_trans );

	nav_msgs::Odometry odom;
	odom.header.stamp = currentTime;
	odom.header.frame_id = ChuckTransforms::ODOMETRY;
	odom.child_frame_id = ChuckTransforms::BASE_FOOTPRINT;

	// Position
	odom.pose.pose.position.x = pose_.x;
	odom.pose.pose.position.y = pose_.y;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = odom_quat;

	// Velocity
	odom.twist.twist.linear.x = linearVelocity;
	odom.twist.twist.linear.y = 0.0;
	odom.twist.twist.linear.z = 0.0;
	odom.twist.twist.angular.x = 0.0;
	odom.twist.twist.angular.y = 0.0;
	odom.twist.twist.angular.z = angularVelocity;

	// Publish the Odometry
	odometryPosePub_.publish( odom );

    // Get and publish the forward estimated velocities
    nav_msgs::Odometry odom_estimate;
    odom_estimate = odom;
    odom_estimate.twist.twist = getEstimatedRobotVel(linearVelocity, angularVelocity, currentTime.toSec());
    odometryPoseEstimatePub_.publish( odom_estimate );

	std_msgs::Bool message;
    message.data = true;

	pingPub_.publish( message );

	// Update the last time
	s_lastTime = currentTime;
	lastPoseTime_ = ros::Time::now();
	s_lastPose = pose_;
}

geometry_msgs::Twist OdometryPositionEstimator::getEstimatedRobotVel(double reported_linear_vel, double reported_angular_vel, double reported_time)
{
    // Estimate current velocities from odometry and command velocities.
    // Copy out data
    geometry_msgs::Twist cmd_vel;
    double cmd_vel_time = 0;
    {
      boost::mutex::scoped_lock lock(cmd_vel_mutex_);
      cmd_vel = cmd_vel_;
      cmd_vel_time = cmd_vel_time_;
    }
    // Get the current time
    double current_time = ros::Time::now().toSec();

    double time_since_measurement = current_time - reported_time; // Unsure if these clocks are the same.

    // Estimate
    geometry_msgs::Twist estimated_vel;
    if ((current_time - cmd_vel_time) > cmd_vel_timeout_)
    {
    	if (cmd_vel_time)
    	{
    		// It's been too long since a cmd vel was sent.  Use the reported velocity.
    		ROS_DEBUG_THROTTLE(1.0f, "cmd vel has a timeout: %f", (current_time - cmd_vel_time));
    	}

    	estimated_vel.linear.x = reported_linear_vel;
		estimated_vel.angular.z = reported_angular_vel;
    }
    else
    {
      double estimate_dt = time_since_measurement + velocity_loop_delays_;
      ROS_DEBUG("Estimate dt: %f", estimate_dt);
      estimated_vel.linear.x = forwardEstimateVelocity(reported_linear_vel, cmd_vel.linear.x, linear_acceleration_rate_, estimate_dt);
      estimated_vel.angular.z = forwardEstimateVelocity(reported_angular_vel, cmd_vel.angular.z, angular_acceleration_rate_, estimate_dt);
      ROS_DEBUG("Vels cmd: [%f, %f], est: [%f, %f], reported: [%f, %f]", cmd_vel.linear.x, cmd_vel.angular.x, estimated_vel.linear.x, reported_linear_vel, estimated_vel.angular.z, reported_angular_vel);
    }
    return estimated_vel;
}

double OdometryPositionEstimator::forwardEstimateVelocity(double old, double cmd, double accel, double dt)
{
    // Make sure that signs work correctly.
    double res = 0;
    if (old > cmd)
    {
      res = std::max(old - accel*dt, cmd);
    }
    else
    {
      res = std::min(old + accel*dt, cmd);
    }
    return res;
}

void OdometryPositionEstimator::ResetOdomPose( const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& assignedPose )
{
	pose_ = PoseMessageFactory::poseStampedWithCovariance2Pose(assignedPose);

	ROS_DEBUG("Robot pose has been set to x= %f, y= %f, theta= %f", pose_.x, pose_.y, pose_.theta);
}

void OdometryPositionEstimator::GetRawOdometryVelocity( const float leftWheelRPM, const float rightWheelRPM, double& linearV, double& angularV)
{
	// Calculate left and right wheel velocity
	double leftWheelVelocity = (double)leftWheelRPM * 2.0 * M_PI * leftWheelRadius_ / 60.0;
	double rightWheelVelocity = (double)rightWheelRPM * 2.0 * M_PI * rightWheelRadius_ / 60.0;

	// Calculate v and w
	linearV = ( rightWheelVelocity + leftWheelVelocity ) / 2.0;
	angularV = ( rightWheelVelocity - leftWheelVelocity ) / wheelbaseLength_ ;
}

void OdometryPositionEstimator::TransformVeclocityToRPM(const geometry_msgs::Twist::ConstPtr& rawVelocity)
{
    ROS_DEBUG_THROTTLE(1.0f, "VelocityToRpm: linear=%f, angular=%f", rawVelocity->linear.x, rawVelocity->angular.z);

	// Whenever Twist command is received, odometry node transforms it to RPM command
	// and sends it to brainstem
	double leftMotorSpeed = rawVelocity->linear.x - ((wheelbaseLength_ / 2) * rawVelocity->angular.z);
	double rightMotorSpeed = rawVelocity->linear.x + ((wheelbaseLength_ / 2) * rawVelocity->angular.z);

	double leftMotorRPM = leftMotorSpeed * 60.0 / 2.0 / M_PI / leftWheelRadius_;
	double rightMotorRPM = rightMotorSpeed * 60.0 / 2.0 / M_PI / rightWheelRadius_;

    ROS_DEBUG_THROTTLE(1.0f, "VelocityToRpm: leftMotorRPM=%f, rightMotorRPM=%f",
    	leftMotorRPM, rightMotorRPM);

	// Publish OdometryRpm message
	srslib_framework::OdometryRpm rpmVelocity;
	rpmVelocity.left_wheel_rpm = leftMotorRPM;
	rpmVelocity.right_wheel_rpm = rightMotorRPM;
	rpmVelocityCmdPub_.publish(rpmVelocity);

  // Also, store it.
  boost::mutex::scoped_lock lock(cmd_vel_mutex_);
  cmd_vel_ = *rawVelocity;
  cmd_vel_time_ = ros::Time::now().toSec();  // Record whatever time it is.
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void OdometryPositionEstimator::pingCallback(const ros::TimerEvent& event)
{
    std_msgs::Bool message;
    message.data = true;

    pingPub_.publish(message);

    double pingDelay = (event.current_real - event.current_expected).toSec();

    // We should never be falling behind by more than 500ms
    if ( pingDelay > (1.0f / PING_HZ) * MAX_ALLOWED_PING_DELAY)
    {
        ROS_ERROR_STREAM( "Motion ping exceeded allowable delay: " << pingDelay );
    }

	double odomDelay = (ros::Time::now() - lastPoseTime_).toSec();

	if (odomDelay > MAX_ALLOWED_ODOM_DELAY) {
		ROS_ERROR_STREAM_THROTTLE( 5.0f, "Odometry topic not published in: " << odomDelay );
	}
}

void OdometryPositionEstimator::cfgCallback(srsnode_odometry::RobotSetupConfig &config, uint32_t level)
{
	wheelbaseLength_ = config.robot_wheelbase_length;
	leftWheelRadius_ = config.robot_leftwheel_radius;
	rightWheelRadius_ = config.robot_rightwheel_radius;
}

} // namespace srs
