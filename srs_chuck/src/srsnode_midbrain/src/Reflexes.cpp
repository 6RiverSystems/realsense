/*
 * Reflex.cpp
 *
 *  Created on: Jun 17, 2016
 *      Author: dan
 */

#include <srsnode_midbrain/Reflexes.hpp>

#include <ros/ros.h>

namespace srs
{

Reflexes::Reflexes(ros::NodeHandle nodeHandle) :
	m_laserScanSubscriber( nodeHandle.subscribe<sensor_msgs::LaserScan>(
        "/camera/scan", 10, std::bind( &Reflexes::OnLaserScan, this, std::placeholders::_1 ) ) )
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
	}
}


} /* namespace srs */
