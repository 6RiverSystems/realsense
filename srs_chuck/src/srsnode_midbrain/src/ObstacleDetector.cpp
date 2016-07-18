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

	double safeDistance = GetSafeDistance( m_linearVelocity );

	std::string strDangerZone;
	std::string strDangerZoneXOnly;
	std::string strNotDanerZone;

	for( int x = 0; x < numberOfScans; x++ )
	{
		double angle = ((double)x * scan->angle_increment) + scan->angle_min;

		double scanDistance = scan->ranges[x];

		if( !std::isinf( scanDistance ) )
		{
			//               fY
			//             ------
			//             \    |
			//              \   |
			// scan distance \  | fX (Distance from chuck)
			//                \0|
			//                 \|

			double fX = scanDistance * cos( angle );
			double fY = scanDistance * sin( angle );

			// TODO: Use polygon trajectory path for more accuracy
			// laser scan in polygon to determine if we have a  hit
			// possibly also use raw point cloud (3d space) to see how
			// many points are in the area (possibly better heuristic)
			if( fX <= safeDistance )
			{
				if( fabs( fY ) <= m_footprint )
				{
					numberOfObstacles++;
				}
			}
		}
	}

//	ROS_DEBUG_THROTTLE_NAMED( 0.3, "reflexes", "linear vel: %0.2f, safeDistance: %.02f, Obstacles detected: %d",
//		m_linearVelocity, safeDistance, numberOfObstacles );

	if( numberOfObstacles > m_objectThreshold )
	{
		if( m_obstacleDetectedCallback )
		{
			m_obstacleDetectedCallback( );
		}
	}
}

double ObstacleDetector::GetSafeDistance( double velocity ) const
{
	double safeDistance = 0.0f;

	if( velocity != 0.0f )
	{
		// Calculated from actual measurements and polynomial regression fitting
		// https://docs.google.com/a/6river.com/spreadsheets/d/1wUPgpESlp-MVbnMGCmFDGH22qyO39D8kzvi7IPZ1Q1c/edit?usp=sharing
		// http://www.xuru.org/rt/PR.asp#CopyPaste
		safeDistance = ( 0.3518981019f * pow( velocity, 2 ) - 0.03806193806f * velocity + 0.01804195804f );
	}

	return safeDistance;
}

double ObstacleDetector::GetFootprint( ) const
{
	return m_footprint;
}

} /* namespace srs */
