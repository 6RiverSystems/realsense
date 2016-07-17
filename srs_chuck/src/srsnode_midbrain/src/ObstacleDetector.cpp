/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#include <srsnode_midbrain/Reflexes.hpp>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tgmath.h>

namespace srs
{

ObstacleDetector::ObstacleDetector( double footprint ) :
	m_obstacleDetectedCallback( ),
	m_linearVelocity( 0.0f ),
	m_angularVelocity( 0.0f ),
	m_objectThreshold( 0 ),
	m_footprint( footprint )
{

}

ObstacleDetector::~ObstacleDetector( )
{

}

void ObstacleDetector::SetDetectionCallback( ObstacleDetectedFn obstacleDetectedCallback )
{
	m_obstacleDetectedCallback = obstacleDetectedCallback;
}

void ObstacleDetector::SetVelocity( double linear, double angular )
{
	m_linearVelocity = linear;

	m_angularVelocity = angular;
}

void ObstacleDetector::SetObjectThreshold( uint32_t objectThreshold )
{
	m_objectThreshold = objectThreshold;
}

void ObstacleDetector::ProcessScan( const sensor_msgs::LaserScan::ConstPtr& scan )
{
	if( m_linearVelocity == 0.0f &&
		m_angularVelocity == 0.0f )
	{
		return;
	}

	uint32_t numberOfObstacles = 0;

	uint32_t numberOfScans = scan->ranges.size( );

	// Calculated from actual measurments and polynomial regression fitting
	// https://docs.google.com/a/6river.com/spreadsheets/d/1wUPgpESlp-MVbnMGCmFDGH22qyO39D8kzvi7IPZ1Q1c/edit?usp=sharing
	// http://www.xuru.org/rt/PR.asp#CopyPaste
	double safeDistance = ( 0.3518981019f * pow( m_linearVelocity, 2 ) - 0.03806193806f * m_linearVelocity + 0.01804195804f ) * 1.1f;

	std::string strPoints;

	char point[255] = { '\0' };
	sprintf( point, "Safe distance (velocity: %.2f): %.2f: ", m_linearVelocity, safeDistance );
	strPoints += point;

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

			sprintf( point, "%.2f, ", fabs( distanceFromBot ));
			strPoints += point;

			if( fabs( distanceFromBot ) < safeDistance )
			{
				if( fabs( yOffset ) < m_footprint )
				{
					numberOfObstacles++;
				}
			}
		}
	}

//	ROS_DEBUG_THROTTLE_NAMED( 0.3, "reflexes", "linear vel: %0.2f, safeDistance: %.02f, Obstacles detected: %d, points: %s",
//		m_linearVelocity, safeDistance, numberOfObstacles, strPoints.c_str( ) );

	if( numberOfObstacles > m_objectThreshold )
	{
		if( m_obstacleDetectedCallback )
		{
			m_obstacleDetectedCallback( numberOfObstacles );
		}

//		// Only send a hard stop if we are paused
//		if( !m_operationalState.pause )
//		{
//
//			if( m_sendHardStop )
//			{
//				// Send the hard stop
//				std_msgs::String msg;
//				msg.data = "STOP";
//
//				m_commandPublisher.publish( msg );
//			}
//		}
	}
}

double ObstacleDetector::GetFootprint( ) const
{
	return m_footprint;
}

} /* namespace srs */
