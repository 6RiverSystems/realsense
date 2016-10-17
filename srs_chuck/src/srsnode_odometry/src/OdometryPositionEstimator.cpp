/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#include <srsnode_odometry/OdometryPositionEstimator.hpp>
#include <srslib_framework/math/TimeMath.hpp>
#include <srslib_framework/math/PoseMath.hpp>

namespace srs {

OdometryPositionEstimator::OdometryPositionEstimator(std::string nodeName) :
	nodeHandle_(nodeName),
	//twist_(),
	pose_(15.711, 5.34, 3.14159),
	pingTimer_(),
	broadcaster_(),
	rawOdometrySub_(),
	odometryPub_(),
	pingPub_(),
	wheelbaseLength_(0.5235),
	leftWheelRadius_(0.10243),
	rightWheelRadius_(0.10243)
{
	configServer_.setCallback(boost::bind(&OdometryPositionEstimator::cfgCallback, this, _1, _2));
}

OdometryPositionEstimator::~OdometryPositionEstimator()
{
}

void OdometryPositionEstimator::run()
{
	ros::Rate refreshRate(REFRESH_RATE_HZ);

	connect();

	while(ros::ok())
	{
		ros::spinOnce();

		refreshRate.sleep();
	}

	disconnect();
}

void OdometryPositionEstimator::connect()
{
	rawOdometrySub_ = nodeHandle_.subscribe<srslib_framework::Odometry>(ODOMETRY_RAW_TOPIC, 10,
		std::bind( &OdometryPositionEstimator::CalculateRobotPose, this, std::placeholders::_1 ));

	odometryPub_ = nodeHandle_.advertise<nav_msgs::Odometry>(ODOMETRY_TOPIC, 100);

	pingPub_ = nodeHandle_.advertise<std_msgs::Bool>("/internal/state/ping", 1);

    // Start the ping timer
    pingTimer_ = nodeHandle_.createTimer(ros::Duration(1.0 / PING_HZ),
        boost::bind(&OdometryPositionEstimator::pingCallback, this, _1));
}

void OdometryPositionEstimator::disconnect()
{
	odometryPub_.shutdown();
}

void OdometryPositionEstimator::CalculateRobotPose( const srslib_framework::Odometry::ConstPtr& encoderCount )
{
	static ros::Time s_lastTime = encoderCount->header.stamp;

	// Update current time and calculate time interval between two consecutive messages
	ros::Time currentTime = encoderCount->header.stamp;

	// Skip first round calculation (dfTimeDelta=0 -> v,w=NaN)
	if(s_lastTime == currentTime)
		return;

	double dfTimeDelta = (currentTime - s_lastTime).toSec();

	// Calculate linear and angular velocity
	double v, w;
	GetRawOdometryVelocity(encoderCount->left_wheel, encoderCount->right_wheel, dfTimeDelta, v, w);

	//static geometry_msgs::Twist s_lastVelocity = twist_;

	// Message declarations
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_footprint";

	/*
	if( s_lastVelocity.twist.linear.x != estimatedVelocity->twist.linear.x ||
		s_lastVelocity.twist.angular.z != estimatedVelocity->twist.angular.z)
	{
		ROS_DEBUG( "Estimated Velocity Changed: linear=%f, angular=%f",
			estimatedVelocity->twist.linear.x, estimatedVelocity->twist.angular.z );
	} */

    //double v = twist_.linear.x;
    //double w = twist_.angular.z;

    constexpr static double ANGULAR_VELOCITY_EPSILON = 0.000001; // [rad/s] (0.0573 [deg/s])

    // Check for the special case in which omega is 0 (the robot is moving straight)
	if (abs(w) > ANGULAR_VELOCITY_EPSILON)
    {
        double r = v / w;

        pose_.x = pose_.x + r * sin(pose_.theta + w * dfTimeDelta) - r * sin(pose_.theta),
        pose_.y = pose_.y + r * cos(pose_.theta) - r * cos(pose_.theta + w * dfTimeDelta),
		pose_.theta = AngleMath::normalizeAngleRad<>(pose_.theta + w * dfTimeDelta);
    }
    else
    {
    	pose_ = PoseMath::translate<>(pose_, v * dfTimeDelta, 0.0);
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
	odom.twist.twist.linear.x = v;
	odom.twist.twist.linear.y = 0.0;
	odom.twist.twist.linear.z = 0.0;
	odom.twist.twist.angular.x = 0.0;
	odom.twist.twist.angular.y = 0.0;
	odom.twist.twist.angular.z = w;

	// Publish the Odometry
	odometryPub_.publish( odom );

	std_msgs::Bool message;
    message.data = true;

	pingPub_.publish( message );

	// Update the last time
	s_lastTime = currentTime;
}

void OdometryPositionEstimator::GetRawOdometryVelocity( const int32_t leftWheelCount, const int32_t rightWheelCount, double timeInterval, double& v, double& w)
{
	constexpr static int MOTOR_COUNT_PER_REVOLUTION = 4096;
	constexpr static int GEAR_BOX_REDUCTION_RATE = 10;

	double leftWheelVelocity = (double)leftWheelCount/ timeInterval / (double)MOTOR_COUNT_PER_REVOLUTION / (double) GEAR_BOX_REDUCTION_RATE * leftWheelRadius_ * 2 * M_PI;
	double rightWheelVelocity = (double)rightWheelCount/ timeInterval / (double)MOTOR_COUNT_PER_REVOLUTION / (double) GEAR_BOX_REDUCTION_RATE * rightWheelRadius_ * 2 * M_PI;

	v = ( rightWheelVelocity + leftWheelVelocity )/2;
	w = ( rightWheelVelocity - leftWheelVelocity )/wheelbaseLength_ ;
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
	wheelbaseLength_ = config.robot_wheelbase_length;
	leftWheelRadius_ = config.robot_leftwheel_radius;
	rightWheelRadius_ = config.robot_rightwheel_radius;
}

} // namespace srs
