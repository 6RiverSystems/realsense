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
	m_chuckWidth_( 0.619125 ),
	m_chuckHalfWidth( m_chuckWidth_ / 2.0f ),
	m_yPaddedOffset( m_chuckHalfWidth * 1.2 ),
	m_safeDistance( 0.5f ),
	m_laserScanSubscriber( nodeHandle.subscribe<sensor_msgs::LaserScan>(
		"/camera/depth/scan", 10, std::bind( &Reflexes::OnLaserScan, this, std::placeholders::_1 ) ) ),
	m_commandPublisher( nodeHandle.advertise<std_msgs::String>(
		"/cmd_ll", 1, true) )
{
	ROS_INFO( "Reflex: half width: %f, yOffset: %f, safeDistance: %f",
		m_chuckHalfWidth, m_yPaddedOffset, m_safeDistance );
}

Reflexes::~Reflexes( )
{

}

void Reflexes::OnLaserScan( const sensor_msgs::LaserScan::ConstPtr& scan )
{
	uint32_t numberOfObstacles = 0;

	uint32_t numberOfScans = scan->ranges.size( );

	for( int x = 0; x < numberOfScans; x++ )
	{
		double angle = ((double)x * scan->angle_increment) + scan->angle_min;

		double scanDistance = scan->ranges[x];

		if( !std::isinf( scanDistance ) )
		{
			//            y Offset
			//             ------
			//             \    |
			//              \   |
			// scan distance \  | Distance from bot
			//                \0|
			//                 \|

			double yOffset = scanDistance * tan( angle );
			double distanceFromBot = yOffset / sin( angle );

			if( abs( distanceFromBot ) < m_safeDistance )
			{
				if( abs( yOffset )  < m_yPaddedOffset )
				{
					numberOfObstacles++;
				}
			}
		}
	}

	if( numberOfObstacles > 10 )
	{
		ROS_DEBUG_THROTTLE( 0.3, "Obstacles detected: %d", numberOfObstacles);

		// Send the hard stop to the brainstem
		std_msgs::String msg;
		msg.data = "STOP";

		m_commandPublisher.publish( msg );
	}
}


} /* namespace srs */
