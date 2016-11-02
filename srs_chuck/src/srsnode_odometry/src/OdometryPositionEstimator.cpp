/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#include <srsnode_odometry/OdometryPositionEstimator.hpp>
#include <srslib_framework/ros/message/PoseMessageFactory.hpp>
#include <srslib_framework/math/TimeMath.hpp>
#include <srslib_framework/math/PoseMath.hpp>

namespace srs {

OdometryPositionEstimator::OdometryPositionEstimator(std::string nodeName) :
	nodeHandle_(nodeName),
	pose_(-1.0, -1.0, -1.0),
	pingTimer_(),
	broadcaster_(),
	rawOdometryCountSub_(),
	odometryPosePub_(),
	pingPub_(),
	motorCountPerRev_(4096),
	gearboxRatio_(10.0),
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

    ROS_INFO_STREAM("Odometry Configuration: motorCountPerRev=" << motorCountPerRev_ <<
		", gearboxRatio_=" << gearboxRatio_ <<
		", wheelbaseLength_=" << wheelbaseLength_ <<
		", leftWheelRadius_=" << leftWheelRadius_ <<
		", rightWheelRadius_=" << rightWheelRadius_);

	while(ros::ok())
	{
		ros::spinOnce();

		refreshRate.sleep();
	}

	disconnect();
}

void OdometryPositionEstimator::connect()
{
	rawOdometryCountSub_ = nodeHandle_.subscribe<srslib_framework::Odometry>(ODOMETRY_RAW_COUNT_TOPIC, 10,
		std::bind( &OdometryPositionEstimator::RawOdomCountToVelocity, this, std::placeholders::_1 ));

	resetPoseSub_ = nodeHandle_.subscribe<geometry_msgs::PoseWithCovarianceStamped>(INITIAL_POSE_TOPIC, 1,
			std::bind( &OdometryPositionEstimator::ResetOdomPose, this, std::placeholders::_1 ));

	odometryPosePub_ = nodeHandle_.advertise<nav_msgs::Odometry>(ODOMETRY_OUTPUT_TOPIC, 100);

	pingPub_ = nodeHandle_.advertise<std_msgs::Bool>("/internal/state/ping", 1);

	// Register dynamic configuration callback
	configServer_.setCallback(boost::bind(&OdometryPositionEstimator::cfgCallback, this, _1, _2));

    // Start the ping timer
    pingTimer_ = nodeHandle_.createTimer(ros::Duration(1.0 / PING_HZ),
        boost::bind(&OdometryPositionEstimator::pingCallback, this, _1));
}

void OdometryPositionEstimator::disconnect()
{
	odometryPosePub_.shutdown();
}

void OdometryPositionEstimator::RawOdomCountToVelocity( const srslib_framework::Odometry::ConstPtr& encoderCount )
{
	// If no initial pose is provided, return immediately without any calculation
	if(pose_.x == (-1.0) && pose_.y == (-1.0) && pose_.theta == (-1.0))
	{
		ROS_ERROR_ONCE("No initial pose provided");
		return;
	}

	static ros::Time s_lastTime = encoderCount->header.stamp;
	static Pose<> s_lastPose = pose_;

	// Update current time and calculate time interval between two consecutive messages
	ros::Time currentTime = encoderCount->header.stamp;

	// Skip first round calculation (if dfTimeDelta = 0 -> v,w = NaN)
	if(!TimeMath::isTimeElapsed(0.00001, s_lastTime, currentTime))
	{
		return;
	}

	double dfTimeDelta = (currentTime - s_lastTime).toSec();

	// Calculate linear and angular velocity
	double linearVelocity = 0.0;
	double angularVelocity = 0.0;
	GetRawOdometryVelocity(encoderCount->left_wheel, encoderCount->right_wheel, dfTimeDelta, linearVelocity, angularVelocity);

	// Message declarations
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_footprint";

    constexpr static double ANGULAR_VELOCITY_EPSILON = 0.000001; // [rad/s] (0.0573 [deg/s])

    // Check for the special case in which omega is 0 (the robot is moving straight)
	if (abs(angularVelocity) > ANGULAR_VELOCITY_EPSILON)
    {
        double r = linearVelocity / angularVelocity;

        pose_.x = pose_.x + r * sin(pose_.theta + angularVelocity * dfTimeDelta) - r * sin(pose_.theta),
        pose_.y = pose_.y + r * cos(pose_.theta) - r * cos(pose_.theta + angularVelocity * dfTimeDelta),
		pose_.theta = AngleMath::normalizeAngleRad<>(pose_.theta + angularVelocity * dfTimeDelta);
    }
    else
    {
    	pose_ = PoseMath::translate<>(pose_, linearVelocity * dfTimeDelta, 0.0);
    }

	// Display status in DEBUG MODE
	if(pose_.x != s_lastPose.x || pose_.y != s_lastPose.y || pose_.theta != s_lastPose.theta)
	{
		ROS_DEBUG( "Estimated Pose Changed: x=%f, y=%f, theta=%f",
					pose_.x, pose_.y, pose_.theta );
	}

//	ROS_ERROR_THROTTLE( 1.0, "Pose: %f, %f", pose_.x, pose_.y );
//	ROS_ERROR_THROTTLE( 1.0, "Pose: x=%f, y=%f, angle:%f", pose_.x, pose_.y, pose_.theta );

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
	odom.header.frame_id = "odom";
	odom.child_frame_id = "base_footprint";

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

	std_msgs::Bool message;
    message.data = true;

	pingPub_.publish( message );

	// Update the last time
	s_lastTime = currentTime;
	s_lastPose = pose_;
}

void OdometryPositionEstimator::ResetOdomPose( const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& assignedPose )
{
	pose_ = PoseMessageFactory::poseStampedWithCovariance2Pose(assignedPose);

	ROS_DEBUG("Robot pose has been set to x= %f, y= %f, theta= %f", pose_.x, pose_.y, pose_.theta);
}

void OdometryPositionEstimator::GetRawOdometryVelocity( const int32_t leftWheelCount, const int32_t rightWheelCount, double timeInterval, double& linearV, double& angularV)
{
	// Calculate left and right wheel velocity
	double leftWheelVelocity = (double)leftWheelCount / timeInterval / (double)motorCountPerRev_ / (double) gearboxRatio_ * leftWheelRadius_ * 2 * M_PI;
	double rightWheelVelocity = (double)rightWheelCount / timeInterval / (double)motorCountPerRev_ / (double) gearboxRatio_ * rightWheelRadius_ * 2 * M_PI;

	// Calculate v and w
	linearV = ( rightWheelVelocity + leftWheelVelocity ) / 2;
	angularV = ( rightWheelVelocity - leftWheelVelocity ) / wheelbaseLength_ ;
}
////////////////////////////////////////////////////////////////////////////////////////////////////
void OdometryPositionEstimator::pingCallback(const ros::TimerEvent& event)
{
    std_msgs::Bool message;
    message.data = true;

    pingPub_.publish(message);

    double delay = (event.current_real - event.current_expected).toSec();

    // We should never be falling behind by more than 500ms
    if ( delay > (1.0f / PING_HZ) * MAX_ALLOWED_PING_DELAY)
    {
        ROS_ERROR_STREAM( "Motion ping exceeded allowable delay: " << delay );
    }
}

void OdometryPositionEstimator::cfgCallback(srsnode_odometry::RobotSetupConfig &config, uint32_t level)
{
	motorCountPerRev_ = config.motor_count_per_rev;
	gearboxRatio_ = config.gearbox_ratio;
	wheelbaseLength_ = config.robot_wheelbase_length;
	leftWheelRadius_ = config.robot_leftwheel_radius;
	rightWheelRadius_ = config.robot_rightwheel_radius;
}

} // namespace srs
