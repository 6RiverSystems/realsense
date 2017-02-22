#include <hw_message/OdometryPoseHandler.hpp>

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_datatypes.h>

#include <srslib_framework/math/BasicMath.hpp>
#include <srslib_framework/math/AngleMath.hpp>
#include <srslib_framework/math/TimeMath.hpp>
#include <srslib_framework/ros/message/VelocityMessageFactory.hpp>
#include <srslib_framework/chuck/ChuckTransforms.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

OdometryPoseHandler::OdometryPoseHandler(PublisherOdometryPose::Interface& publisher, bool useBrainstemOdom) :
	HardwareMessageHandler(BRAIN_STEM_MSG::ODOMETRY_POSE),
	publisher_(publisher),
	useBrainstemOdom_(useBrainstemOdom),
	tempRobotPose_(),
	offsetRobotPose_()
{

}

////////////////////////////////////////////////////////////////////////////////////////////////////
void OdometryPoseHandler::receiveMessage(ros::Time currentTime, HardwareMessage& msg)
{
	OdometryPoseData odometryPoseData = msg.read<OdometryPoseData>();

	// store pose information while receiving message
	tempRobotPose_ = odometryPoseData;

	nav_msgs::Odometry odom;
	odom.header.stamp = currentTime;
	odom.header.frame_id = ChuckTransforms::ODOMETRY;
	odom.child_frame_id = ChuckTransforms::BASE_FOOTPRINT;

	// rather than publish odometryPoseData directly, add the pose offset value instead
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw( odometryPoseData.theta + offsetRobotPose_.theta );

	float theta = offsetRobotPose_.theta * (-1);
	// Position
	// similar idea as handlePoseReset() transforming odometryPoseData into world frame
	odom.pose.pose.position.x = odometryPoseData.x * cos(theta) + odometryPoseData.y * sin(theta) + offsetRobotPose_.x;
	odom.pose.pose.position.y = (-1) * odometryPoseData.x * sin(theta) + odometryPoseData.y * cos(theta) + offsetRobotPose_.y;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = odom_quat;

	// Velocity
	odom.twist.twist.linear.x = odometryPoseData.linearVelocity;
	odom.twist.twist.linear.y = 0.0;
	odom.twist.twist.linear.z = 0.0;
	odom.twist.twist.angular.x = 0.0;
	odom.twist.twist.angular.y = 0.0;
	odom.twist.twist.angular.z = odometryPoseData.angularVelocity;

	publisher_.publish(odom);

	if (useBrainstemOdom_)
	{
		// Publish the TF
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.frame_id = ChuckTransforms::ODOMETRY;
		odom_trans.child_frame_id = ChuckTransforms::BASE_FOOTPRINT;

		odom_trans.header.stamp = currentTime;
		odom_trans.transform.translation.x = odometryPoseData.linearVelocity;
		odom_trans.transform.translation.y = odometryPoseData.angularVelocity;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = odom_quat;

		broadcaster_.sendTransform( odom_trans );
	}
}

void OdometryPoseHandler::handlePoseReset()
{
	// accumulate the offset each time when disconnect happen
	// since brainstem reset to (x:0, y:0, theta:0) every time, we need to
	// transform the pose into world frame

	// the negtive sign is for projecting current pose frame back to world frame
	float theta = offsetRobotPose_.theta * (-1);
	offsetRobotPose_.x += (tempRobotPose_.x * cos(theta) + tempRobotPose_.y * sin(theta));
	offsetRobotPose_.y += ((-1) * tempRobotPose_.x * sin(theta) + tempRobotPose_.y * cos(theta));
	offsetRobotPose_.theta += tempRobotPose_.theta;
}

} // namespace srs
