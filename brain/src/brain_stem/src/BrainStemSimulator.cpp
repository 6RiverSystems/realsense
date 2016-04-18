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

		ROS_DEBUG_THROTTLE( 5.0f, "Simulated Velocity Changed: linear=%f, angular=%f",
			g_velocity.linear.x, g_velocity.angular.z );
	}
}

void RawOdometryVelocity( const geometry_msgs::TwistStamped& estimatedVelocity )
{
	if( !g_bUseEstimatedVelocity )
	{
		ROS_DEBUG( "Switching to actual estimated velocity: " );

		g_bUseEstimatedVelocity = true;
	}

	g_velocity = estimatedVelocity.twist;

	ROS_DEBUG_THROTTLE( 5.0f, "Estimated Velocity Changed: linear=%f, angular=%f",
		g_velocity.linear.x, g_velocity.angular.z );
}

int main(int argc, char** argv) {

	ros::init(argc, argv, "brain_stem_simulator");
	ros::NodeHandle node;

	ros::Subscriber commandVelocitySub = node.subscribe( "/cmd_vel", 1000, OnCommandVelocity );

	ros::Subscriber rawOdometrySub = node.subscribe( "/sensors/odometry/raw", 1000, OnCommandVelocity );

	ros::Publisher odom_pub = node.advertise<nav_msgs::Odometry>("/odom", 50);

	tf::TransformBroadcaster broadcaster;
	ros::Rate loop_rate(20);

	// initial position
	double x = 2.0;
	double y = 2.0;
	double th = 0.0;

	ros::Time current_time;
	ros::Time last_time;
	current_time = ros::Time::now( );
	last_time = ros::Time::now( );

	// message declarations
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_footprint";

	while( ros::ok( ) )
	{
		ros::spinOnce( );

		current_time = ros::Time::now();

		double dt = (current_time - last_time).toSec();
		double delta_x = g_velocity.linear.x * cos(th) * dt;
		double delta_y = g_velocity.linear.x * sin(th) * dt;
		double delta_th = g_velocity.angular.z* dt;

		x += delta_x;
		y += delta_y;
		th += delta_th;

		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

		// Publish the TF
		odom_trans.header.stamp = current_time;
		odom_trans.transform.translation.x = x;
		odom_trans.transform.translation.y = y;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = odom_quat;

		broadcaster.sendTransform(odom_trans);

		nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
		odom.header.frame_id = "odom";
		odom.child_frame_id = "base_footprint";

		// Position
		odom.pose.pose.position.x = x;
		odom.pose.pose.position.y = y;
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
		odom_pub.publish(odom);

		last_time = current_time;

		loop_rate.sleep( );
	}
	return 0;
}
