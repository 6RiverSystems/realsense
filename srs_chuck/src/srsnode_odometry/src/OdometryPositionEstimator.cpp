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
	twist_(),
	pose_(15.71100, 5.33400, M_PI),
	pingTimer_(),
	broadcaster_(),
	rawOdometrySub_(),
	odometryPub_(),
	pingPub_()
{

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
	rawOdometrySub_ = nodeHandle_.subscribe<geometry_msgs::TwistStamped>(ODOMETRY_RAW_TOPIC, 10,
		std::bind( &OdometryPositionEstimator::RawOdometryVelocity, this, std::placeholders::_1 ));

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

void OdometryPositionEstimator::RawOdometryVelocity( const geometry_msgs::TwistStamped::ConstPtr& estimatedVelocity )
{
	static ros::Time s_lastTime = estimatedVelocity->header.stamp;
	static geometry_msgs::TwistStamped s_lastVelocity = *estimatedVelocity;

	// Message declarations
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_footprint";

	if( s_lastVelocity.twist.linear.x != estimatedVelocity->twist.linear.x ||
		s_lastVelocity.twist.angular.z != estimatedVelocity->twist.angular.z)
	{
		ROS_DEBUG( "Estimated Velocity Changed: linear=%f, angular=%f",
			estimatedVelocity->twist.linear.x, estimatedVelocity->twist.angular.z );
	}

	twist_ = estimatedVelocity->twist;

	ros::Time currentTime = estimatedVelocity->header.stamp;

    double v = twist_.linear.x;
    double w = twist_.angular.z;

    constexpr static double ANGULAR_VELOCITY_EPSILON = 0.000001; // [rad/s] (0.0573 [deg/s])

	double dfTimeDelta = (currentTime - s_lastTime).toSec( );

    // Check for the special case in which omega is 0 (the robot is moving straight)
	if (abs(w) > ANGULAR_VELOCITY_EPSILON)
    {
        double r = v / w;

        pose_.x + r * sin(pose_.theta + w * dfTimeDelta) - r * sin(pose_.theta),
        pose_.y + r * cos(pose_.theta) - r * cos(pose_.theta + w * dfTimeDelta),
        AngleMath::normalizeAngleRad<>(pose_.theta + w * dfTimeDelta);
    }
    else
    {
    	pose_ = PoseMath::translate<>(pose_, v * dfTimeDelta, 0.0);
    }

	ROS_DEBUG_THROTTLE_NAMED( 1.0f, "OdomEstimator", "Pose Changed: x=%lf, y=%lf, theta=%lf",
		pose_.x, pose_.y, pose_.theta * 180.0f / M_PI );

	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw( pose_.theta );

	// Publish the TF
	odom_trans.header.stamp = ros::Time::now( );
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
	odom.twist.twist.linear.x = twist_.linear.x;
	odom.twist.twist.linear.y = 0.0;
	odom.twist.twist.linear.z = 0.0;
	odom.twist.twist.angular.x = 0.0;
	odom.twist.twist.angular.y = 0.0;
	odom.twist.twist.angular.z = twist_.angular.z;

	// Publish the Odometry
	odometryPub_.publish( odom );

    std_msgs::Bool message;
    message.data = true;

	pingPub_.publish( message );

	// Update the last time
	s_lastTime = currentTime;
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
        ROS_ERROR_STREAM_NAMED("motion", "Motion ping exceeded allowable delay: " << delay );
    }
}


} // namespace srs
