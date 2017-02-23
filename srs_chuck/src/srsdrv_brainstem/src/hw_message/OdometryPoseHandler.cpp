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
	useBrainstemOdom_(useBrainstemOdom)
{
	globalTransform_.setIdentity();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void OdometryPoseHandler::receiveMessage(ros::Time currentTime, HardwareMessage& msg)
{
	OdometryPoseData odometryPoseData = msg.read<OdometryPoseData>();

	// convert odometryPoseData into a tf::Transform
	tf::Vector3 translation(odometryPoseData.x, odometryPoseData.y, 0.0);
	tf::Quaternion rotation = tf::createQuaternionFromYaw(odometryPoseData.theta);
	tempTransform_.setOrigin(translation);
	tempTransform_.setRotation(rotation);

	// calculate current pose
	// use StampedTransform because it can be converted to geometry_msgs::TransformStamped directly
	tf::StampedTransform currentTransform( globalTransform_ * tempTransform_, currentTime, ChuckTransforms::ODOMETRY, ChuckTransforms::BASE_FOOTPRINT);

	// publish pose information
	nav_msgs::Odometry odom;
	odom.header.stamp = currentTime;
	odom.header.frame_id = ChuckTransforms::ODOMETRY;
	odom.child_frame_id = ChuckTransforms::BASE_FOOTPRINT;

	// convert tf::Quaternion to nav_msgs::Odometry Quaternion
	geometry_msgs::Quaternion odom_quat;
	tf::quaternionTFToMsg(currentTransform.getRotation(), odom_quat);

	// Position
	odom.pose.pose.position.x = currentTransform.getOrigin().x();
	odom.pose.pose.position.y = currentTransform.getOrigin().y();
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
		tf::transformStampedTFToMsg(currentTransform, odom_trans);

		broadcaster_.sendTransform( odom_trans );
	}
}

void OdometryPoseHandler::handlePoseReset()
{
	// update the globalTransform when brainstem reset happens
	globalTransform_ *= tempTransform_;
}

} // namespace srs
