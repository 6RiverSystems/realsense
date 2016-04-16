#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

double vel_x = 0.0f;
double vel_th = 0.0f;

void cmd_velCallback(const geometry_msgs::Twist &twist_aux)
{
	vel_x = twist_aux.linear.x;
	vel_th = twist_aux.angular.z;

	ROS_ERROR( "Velocity changed: linear_x=%f, linear_y=%f, linear_x=%f, angular_x=%f, angular_y=%f, angular_z=%f",
		twist_aux.linear.x, twist_aux.linear.y, twist_aux.linear.z, twist_aux.angular.x,
		twist_aux.angular.y, twist_aux.angular.z);
}

int main(int argc, char** argv) {

	ros::init(argc, argv, "brain_stem_simulator");
	ros::NodeHandle n;

	ros::Subscriber cmd_vel_sub = n.subscribe("/cmd_vel", 1000, cmd_velCallback);
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/odom", 50);

	tf::TransformBroadcaster broadcaster;
	ros::Rate loop_rate(20);

	// initial position
	double x = 1.0;
	double y = 1.0;
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
		double delta_x = vel_x * cos(th) * dt;
		double delta_y = vel_x * sin(th) * dt;
		double delta_th = vel_th * dt;

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
		odom.twist.twist.linear.x = vel_x;
		odom.twist.twist.linear.y = 0.0;
		odom.twist.twist.linear.z = 0.0;
		odom.twist.twist.angular.x = 0.0;
		odom.twist.twist.angular.y = 0.0;
		odom.twist.twist.angular.z = vel_th;

		if( delta_x ||
			delta_y ||
			delta_th )
		{
			ROS_ERROR( "Moving chuck by: x=%f, y=%f, angle=%f", delta_x, delta_y, delta_th);
			ROS_ERROR( "Moving chuck: x=%f, y=%f, angle=%f", x, y, th);
		}

		// publishing the odometry and the new tf
		odom_pub.publish(odom);

		last_time = current_time;

		loop_rate.sleep( );
	}
	return 0;
}
