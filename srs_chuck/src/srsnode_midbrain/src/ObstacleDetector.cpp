/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#include <srsnode_midbrain/Reflexes.hpp>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tgmath.h>

std::ostream& operator<<( std::ostream& os, const Segment& segment )
{
	os << segment.first.x << ", " << segment.first.y;
	os << " - ";
	os << segment.second.x << ", " << segment.second.y;

	return os;
}

std::ostream& operator<<( std::ostream& os, const Polygon& polygon )
{
	os << "(";

	bool first = true;

	for( auto point : polygon )
	{
		if( !first )
		{
			os << " => ";
		}

		os << point.x << ", " << point.y;

		first = false;
	}

	os << ")";

	return os;
}

namespace srs
{

ObstacleDetector::ObstacleDetector( double footprint ) :
	m_obstacleDetectedCallback( ),
	m_linearVelocity( 0.0f ),
	m_angularVelocity( 0.0f ),
	m_irThreshold( 0 ),
	m_depthThreshold( 0 ),
	m_footprint( footprint ),
	m_testMode( false )
{

}

ObstacleDetector::~ObstacleDetector( )
{

}

void ObstacleDetector::SetDetectionCallback( ObstacleDetectedFn obstacleDetectedCallback )
{
	m_obstacleDetectedCallback = obstacleDetectedCallback;
}

void ObstacleDetector::SetTestMode( bool testMode )
{
	m_testMode = testMode;
}

bool ObstacleDetector::GetTestMode( ) const
{
	return m_testMode;
}

void ObstacleDetector::SetVelocity( double linear, double angular )
{
	m_linearVelocity = linear;

	m_angularVelocity = angular;
}

void ObstacleDetector::SetThreshold( uint32_t irThreshold, uint32_t depthThreshold )
{
	m_irThreshold = irThreshold;
	m_depthThreshold = depthThreshold;
}

void ObstacleDetector::ProcessScan( const sensor_msgs::LaserScan::ConstPtr& scan, bool isIrScan )
{
	if( !GetTestMode( ) &&
		m_linearVelocity == 0.0f &&
	 	m_angularVelocity == 0.0f )
	 {
	 	return;
	 }

	uint32_t threshold = isIrScan ? m_irThreshold : m_depthThreshold;

	uint32_t numberOfObstacles = 0;

	uint32_t numberOfScans = scan->ranges.size( );

	double safeDistance = GetSafeDistance( m_linearVelocity );

	std::string strPoints;
	char pszPoints[255] = { '\0' };


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
					sprintf( pszPoints, "%.1f, ", scanDistance );
					strPoints += pszPoints;

					numberOfObstacles++;
				}
			}
		}
	}

	if( numberOfObstacles > threshold )
	{
		ROS_ERROR_NAMED( "obstacle_detection", "%s Scan:, scans: %d, linear vel: %0.2f, m_footprint: %0.2f, " \
			"safeDistance: %.02f, Obstacles detected: %d, points: %s", isIrScan ? "IR" : "Depth", numberOfScans,
			m_linearVelocity, m_footprint, safeDistance, numberOfObstacles, strPoints.c_str( ) );

		if( m_obstacleDetectedCallback )
		{
			m_obstacleDetectedCallback( );
		}
	}
	else
	{
		if( strPoints.size( ) )
		{
			ROS_DEBUG_THROTTLE_NAMED( 1.0, "obstacle_detection", "%s Scan:, scans: %d, linear vel: %0.2f, m_footprint: %0.2f, " \
				"safeDistance: %.02f, Obstacles detected: %d, points: %s", isIrScan ? "IR" : "Depth", numberOfScans,
				m_linearVelocity, m_footprint, safeDistance, numberOfObstacles, strPoints.c_str( ) );
		}
	}
}

double ObstacleDetector::GetSafeDistance( double velocity ) const
{
	double safeDistance = 0.0f;

	if( velocity >= 0.1f ||
		GetTestMode( ) )
	{
		// Calculated from actual measurements and polynomial regression fitting
		// https://docs.google.com/a/6river.com/spreadsheets/d/1wUPgpESlp-MVbnMGCmFDGH22qyO39D8kzvi7IPZ1Q1c/edit?usp=sharing
		// http://www.xuru.org/rt/PR.asp#CopyPaste
		safeDistance = ( 0.3518981019f * pow( velocity, 2 ) - 0.03806193806f * velocity + 0.01804195804f ) * 1.2;

		safeDistance = std::max( safeDistance, 0.45 );
	}

	return safeDistance;
}

double ObstacleDetector::GetFootprint( ) const
{
	return m_footprint;
}

Polygon ObstacleDetector::GetDangerZone( ) const
{
	double safeDistance = GetSafeDistance( m_linearVelocity );

	Polygon dangerZone({
		{ 0.0, -m_footprint },
		{ safeDistance, -m_footprint },
		{ safeDistance, m_footprint },
		{ 0.0, m_footprint }
	});

	return dangerZone;

}

} /* namespace srs */
