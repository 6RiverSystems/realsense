#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

geometry_msgs::Twist g_twist;

void cmd_velCallback( const geometry_msgs::Twist& twist )
{
	g_twist = twist;

	ROS_DEBUG_THROTTLE( 5.0f, "Velocity changed: linear_x=%f, linear_y=%f, linear_x=%f, angular_x=%f, angular_y=%f, angular_z=%f",
		twist.linear.x, twist.linear.y, twist.linear.z, twist.angular.x,
		twist.angular.y, twist.angular.z );
}

int main(int argc, char** argv) {

	ros::init(argc, argv, "brain_stem_simulator");
	ros::NodeHandle n;

	ros::Subscriber cmd_vel_sub = n.subscribe("/cmd_vel", 1000, cmd_velCallback);
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/odom", 50);

	tf::TransformBroadcaster broadcaster;
	ros::Rate loop_rate(20);

	// initial position
	double x = 2.0;
	double y = 2.0;
	double th = 0.0;

	ros::Time current_time;
	ros::Time last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();

	// message declarations
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_footprint";

	while( ros::ok( ) )
	{
		ros::spinOnce( );

		current_time = ros::Time::now();

		double dt = (current_time - last_time).toSec();
		double delta_x = g_twist.linear.x * cos(th) * dt;
		double delta_y = g_twist.linear.x * sin(th) * dt;
		double delta_th = g_twist.angular.z* dt;

		x += delta_x;
		y += delta_y;
		th += delta_th;

		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

		// update transform
		odom_trans.header.stamp = current_time;
		odom_trans.transform.translation.x = x;
		odom_trans.transform.translation.y = y;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = odom_quat;

		broadcaster.sendTransform(odom_trans);

		//filling the odometry
		nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
		odom.header.frame_id = "odom";
		odom.child_frame_id = "base_footprint";

		// position
		odom.pose.pose.position.x = x;
		odom.pose.pose.position.y = y;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;

		//velocity
		odom.twist.twist.linear.x = g_twist.linear.x;
		odom.twist.twist.linear.y = 0.0;
		odom.twist.twist.linear.z = 0.0;
		odom.twist.twist.angular.x = 0.0;
		odom.twist.twist.angular.y = 0.0;
		odom.twist.twist.angular.z = g_twist.angular.z;

//		if( delta_x ||
//			delta_y ||
//			delta_th )
//		{
//			ROS_ERROR( "Moving chuck by: x=%f, y=%f, angle=%f", delta_x, delta_y, delta_th);
//			ROS_ERROR( "Moving chuck: x=%f, y=%f, angle=%f", x, y, th);
//		}

		// publishing the odometry and the new tf
		odom_pub.publish(odom);

		last_time = current_time;

		loop_rate.sleep( );
	}
	return 0;
}
