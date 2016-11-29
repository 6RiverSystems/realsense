/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#include <srsnode_midbrain/Reflexes.hpp>

#include <costmap_2d/footprint.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <std_msgs/String.h>

#include <geometry_msgs/PolygonStamped.h>
#include <tgmath.h>
#include <boost/range/adaptor/reversed.hpp>

#include <srslib_framework/ros/message/PolygonMessageFactory.hpp>

namespace srs
{

Reflexes::Reflexes()
{
	readParams();
}

Reflexes::~Reflexes( )
{

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// New code
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Reflexes::readParams()
{
	ROS_INFO("Getting footprint");
	// Get the footprint
	ros::NodeHandle nh("/move_base/local_costmap");
	std::string nm("foo");
	nh.searchParam("footprint", nm);

	ROS_INFO("NH Name: %s, Footprint name: %s", nh.getNamespace().c_str(), nm.c_str());
	std::vector<geometry_msgs::Point> footprint = costmap_2d::makeFootprintFromParams(nh);
	hardStopReflex_.setFootprint(PolygonMessageFactory::points2Poses(footprint));
	ROS_INFO("Got footprint?");
}

void Reflexes::execute()
{
	// ROS_INFO("Executing");
	// Get the sensor position
	hardStopReflex_.setLidarPose(tapLidarPoseOnRobot_.pop());

	// ROS_INFO("Got lidar pose");
	// Update robot position
	hardStopReflex_.setPose(tapRobotPose_.pop());

	// Check for laser scan.
	if (tapFilteredLidar_.newDataAvailable())
	{
		hardStopReflex_.setLaserScan(tapFilteredLidar_.pop());
	}

	// Check for velocity.
	if (tapOdometryPose_.newDataAvailable())
	{
		hardStopReflex_.setVelocity(tapOdometryPose_.popVelocity());
	}

	// Call for hard stop check.
	if (hardStopReflex_.checkHardStop())
	{
		ROS_INFO("Publishing stop");
		cmdClChannel_.publish("STOP;");
	}
	else if (hardStopReflex_.checkForClear())
	{
		ROS_INFO("Clearing motion.");
		cmdClChannel_.publish("CLEAR_MOTION_STATUS;");
	}
	// ROS_INFO("ready to get danger zone.");

	// Publish the polygon
	dangerZoneChannel_.publish(hardStopReflex_.getDangerZoneForDisplay());
	// ROS_INFO("Finished execute loop");
}

} /* namespace srs */

