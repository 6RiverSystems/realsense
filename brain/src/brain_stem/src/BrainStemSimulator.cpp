#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

geometry_msgs::Twist g_velocity;

bool g_bUseEstimatedVelocity = false;

void OnCommandVelocity( const geometry_msgs::Twist& desiredVelocity )
{
	if( !g_bUseEstimatedVelocity )
	{
		g_velocity = desiredVelocity;
	}

	ROS_DEBUG_THROTTLE( 1.0f, "Desired Velocity Changed: linear=%f, angular=%f",
		g_velocity.linear.x, g_velocity.angular.z );
}

void RawOdometryVelocity( const geometry_msgs::TwistStamped& estimatedVelocity )
{
	if( !g_bUseEstimatedVelocity )
	{
		g_bUseEstimatedVelocity = true;
	}

	if( g_velocity.linear.x != estimatedVelocity.twist.linear.x  ||
		g_velocity.angular.z != estimatedVelocity.twist.angular.z )
	{
		ROS_DEBUG_THROTTLE( 1.0f, "Estimated Velocity Changed: linear=%f, angular=%f",
				estimatedVelocity.twist.linear.x, estimatedVelocity.twist.angular.z );
	}

	g_velocity = estimatedVelocity.twist;
}

int main(int argc, char** argv) {

	ros::init(argc, argv, "brain_stem_simulator");
	ros::NodeHandle node;

	ros::Subscriber commandVelocitySub = node.subscribe( "/cmd_vel", 1000, OnCommandVelocity );

	ros::Subscriber rawOdometrySub = node.subscribe( "/sensors/odometry/raw", 1000, RawOdometryVelocity );

	ros::Publisher odom_pub = node.advertise<nav_msgs::Odometry>("/odom", 50);

	tf::TransformBroadcaster broadcaster;
	ros::Rate loop_rate( 50 );

	// initial position
	double dfX = 2.5;
	double dfY = 1.5;
	double dfTheta = 0.0;

	ros::Time current_time;
	ros::Time last_time;
	current_time = ros::Time::now( );
	last_time = ros::Time::now( );

	// message declarations
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_footprint";

	g_velocity.linear.x = 0;
	g_velocity.angular.z = 0;

	while( ros::ok( ) )
	{
		ros::spinOnce( );

		current_time = ros::Time::now( );

		double dfTimeDelta = (current_time - last_time).toSec( );
		double dfXDelta = g_velocity.linear.x * cos( dfTheta ) * dfTimeDelta;
		double dfYDelta = g_velocity.linear.x * sin( dfTheta ) * dfTimeDelta;
		double dfThetaDelta = g_velocity.angular.z* dfTimeDelta;

		dfX += dfXDelta;
		dfY += dfYDelta;
		dfTheta += dfThetaDelta;

		if( dfXDelta || dfYDelta || dfThetaDelta )
		{
			ROS_DEBUG_THROTTLE( 1.0f, "Pose Changed: x=%lf, y=%lf, theta=%lf",
				dfX, dfY, dfTheta );
		}

		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw( dfTheta );

		// Publish the TF
		odom_trans.header.stamp = current_time;
		odom_trans.transform.translation.x = dfX;
		odom_trans.transform.translation.y = dfY;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = odom_quat;

		broadcaster.sendTransform( odom_trans );

		nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
		odom.header.frame_id = "odom";
		odom.child_frame_id = "base_footprint";

		// Position
		odom.pose.pose.position.x = dfX;
		odom.pose.pose.position.y = dfY;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;

		// Velocity
		odom.twist.twist.linear.x = g_velocity.linear.x;
		odom.twist.twist.linear.y = 0.0;
		odom.twist.twist.linear.z = 0.0;
		odom.twist.twist.angular.x = 0.0;
		odom.twist.twist.angular.y = 0.0;
		odom.twist.twist.angular.z = g_velocity.angular.z;

		// Publish the Odometry
		odom_pub.publish( odom );

		last_time = current_time;

		loop_rate.sleep( );
	}
	return 0;
}
