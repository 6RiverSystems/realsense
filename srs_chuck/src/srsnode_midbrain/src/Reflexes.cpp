/*
 * Reflex.cpp
 *
 *  Created on: Jun 17, 2016
 *      Author: dan
 */

#include <srsnode_midbrain/Reflexes.hpp>

#include <ros/ros.h>
#include <std_msgs/String.h>

namespace srs
{

Reflexes::Reflexes(ros::NodeHandle nodeHandle) :
	m_laserScanSubscriber( nodeHandle.subscribe<sensor_msgs::LaserScan>(
		"/camera/scan", 10, std::bind( &Reflexes::OnLaserScan, this, std::placeholders::_1 ) ) ),
	m_commandPublisher( nodeHandle.advertise<std_msgs::String>(
		"/cmd_ll", 1, true) )
{

}

Reflexes::~Reflexes( )
{

}

void Reflexes::OnLaserScan( const sensor_msgs::LaserScan::ConstPtr& scan )
{
	for( int x = scan->range_min; x < scan->range_max; x++ )
	{
		double angle = (double)x * scan->angle_increment;

		double scanDistance = scan->ranges[x];

		// TODO: Parse from URDF or pass through from the executive
		double chuckWidth = 0.619125;
		double chuckHalfWidth = chuckWidth / 2.0f;

		double yOffsetWithBuffer = chuckHalfWidth * 1.2f;
		double safeDistanceFromBot = 0.5f;

		//            y Offset
		//             ------
		//             \    |
		//              \   |
		// scan distance \  | Distance from bot
		//                \0|
		//                 \|

		double yOffset = scanDistance * tan( angle );
		double distanceFromBot = distanceFromBot / sin( angle );

		ROS_DEBUG( "Reflex: yOffset: %f, distance: %f", yOffset, distanceFromBot);

		if( yOffset <= yOffsetWithBuffer &&
			distanceFromBot < safeDistanceFromBot)
		{
			ROS_DEBUG( "Reflex detected obstacle" );

			// Send the hard stop to the brainstem
			std_msgs::String msg;
			msg.data = "STOP";

			m_commandPublisher.publish( msg );
		}
	}
}


} /* namespace srs */
