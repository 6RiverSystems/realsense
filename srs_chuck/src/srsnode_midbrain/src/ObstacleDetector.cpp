/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#include <srsnode_midbrain/Reflexes.hpp>
#include <srslib_framework/math/PoseMath.hpp>
#include <srslib_framework/ros/message/PoseMessageFactory.hpp>

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
	m_pose( ),
	m_solution( ),
	m_dangerZone( ),
	m_linearVelocity( 0.0f ),
	m_angularVelocity( 0.0f ),
	m_depthThreshold( 0 ),
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

void ObstacleDetector::SetPose( const srslib_framework::MsgPose::ConstPtr& pose )
{
	m_pose = *pose;

	UpdateDangerZone( );
}

void ObstacleDetector::SetSolution( const srslib_framework::MsgSolution::ConstPtr& solution )
{
	m_solution = *solution;

	UpdateDangerZone( );
}

void ObstacleDetector::SetThreshold( uint32_t depthThreshold )
{
	m_depthThreshold = depthThreshold;
}

void ObstacleDetector::ProcessScan( const sensor_msgs::LaserScan::ConstPtr& scan, bool isIrScan )
{
	uint32_t threshold = m_depthThreshold;

	uint32_t numberOfObstacles = 0;

	uint32_t numberOfScans = scan->ranges.size( );

	double safeDistance = GetSafeDistance( m_linearVelocity, m_angularVelocity );

	std::string strObstaclePoints;
	std::string strValidPoints;
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
					sprintf( pszPoints, "%.2f, ", scanDistance );
					strObstaclePoints += pszPoints;

					numberOfObstacles++;
				}
				else
				{
					sprintf( pszPoints, "%.2f, ", scanDistance );
					strValidPoints += pszPoints;
				}
			}
			else
			{
				sprintf( pszPoints, "%.2f, ", scanDistance );
				strValidPoints += pszPoints;
			}
		}
	}

	if( numberOfObstacles > threshold )
	{
		ROS_INFO_NAMED( "obstacle_detection", "Scans: %d, linear vel: %0.2f, angular vel: %0.2f, m_footprint: %0.2f, safeDistance: %.02f, Obstacles detected: %d, points: %s", numberOfScans, m_linearVelocity, m_angularVelocity, m_footprint, safeDistance, numberOfObstacles, strObstaclePoints.c_str( ) );

		if( m_obstacleDetectedCallback )
		{
			m_obstacleDetectedCallback( );
		}
	}
	else
	{
		if( strValidPoints.size( ) )
		{
			ROS_DEBUG_THROTTLE_NAMED( 1.0, "obstacle_detection", "Scans: %d, linear vel: %0.2f, angular vel: %0.2f, m_footprint: %0.2f, safeDistance: %.02f, Objects detected: %d, points: %s",
				numberOfScans, m_linearVelocity, m_angularVelocity, m_footprint, safeDistance, numberOfObstacles, strValidPoints.c_str( ) );
		}
	}
}

double ObstacleDetector::GetSafeDistance( double linearVelocity, double angularVelocity ) const
{
	double safeDistance = 0.0f;

	if( linearVelocity > 0.1 && angularVelocity < .1 )
	{

		// Calculated from actual measurements and polynomial regression fitting
		// https://docs.google.com/a/6river.com/spreadsheets/d/1wUPgpESlp-MVbnMGCmFDGH2/2qyO39D8kzvi7IPZ1Q1c/edit?usp=sharing
		// http://www.xuru.org/rt/PR.asp#CopyPaste
		double calculatedSafeDistance = ( 0.3518981019f * pow( linearVelocity, 2 ) - 0.03806193806f * linearVelocity + 0.01804195804f ) * 1.5;

		safeDistance = std::max( calculatedSafeDistance, 0.45 );

		ROS_DEBUG_THROTTLE_NAMED( 1.0f, "obstacle_detection", "calculatedSafeDistance: %0.2f, safeDistance: %0.2f", calculatedSafeDistance, safeDistance ); 
	}

	return safeDistance;
}

double ObstacleDetector::GetFootprint( ) const
{
	return m_footprint;
}

Polygon ObstacleDetector::GetDangerZone( ) const
{
	double safeDistance = GetSafeDistance( m_linearVelocity, m_angularVelocity );

	Polygon dangerZone({
		{ 0.0, -m_footprint },
		{ safeDistance, -m_footprint },
		{ safeDistance, m_footprint },
		{ 0.0, m_footprint }
	});

	return dangerZone;
}

void ObstacleDetector::UpdateDangerZone( )
{
	Polygon dangerZone;

	if( m_solution.items.size( ) )
	{
		// Build the danger zone from the solution
		for( auto solutionItem : m_solution.items )
		{
			std::vector<Pose<>> targetArea = PoseMath::pose2polygon(PoseMessageFactory::msg2Pose(solutionItem.toPose), 0.0, 0.2, 0.2);
		}
	}

	m_dangerZone = dangerZone;
}

} /* namespace srs */
