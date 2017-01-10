#include <hw_message/OdometryPoseHandler.hpp>

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_datatypes.h>

#include <srslib_framework/math/BasicMath.hpp>
#include <srslib_framework/math/AngleMath.hpp>
#include <srslib_framework/math/TimeMath.hpp>
#include <srslib_framework/ros/message/VelocityMessageFactory.hpp>
#include <srslib_framework/ros/topics/ChuckTransforms.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

OdometryPoseHandler::OdometryPoseHandler(ChannelBrainstemOdometryPose::Interface& publisher) :
	HardwareMessageHandler(BRAIN_STEM_MSG::ODOMETRY_POSE),
	publisher_(publisher)
{

}

////////////////////////////////////////////////////////////////////////////////////////////////////
void OdometryPoseHandler::receiveMessage(ros::Time currentTime, HardwareMessage& msg)
{
	OdometryPoseData odometryPoseData = msg.read<OdometryPoseData>();

	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw( odometryPoseData.theta );

	nav_msgs::Odometry odom;
	odom.header.stamp = currentTime;
	odom.header.frame_id = ChuckTransforms::ODOMETRY;
	odom.child_frame_id = ChuckTransforms::BASE_FOOTPRINT;

	// Position
	odom.pose.pose.position.x = odometryPoseData.x;
	odom.pose.pose.position.y = odometryPoseData.y;
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

} // namespace srs
