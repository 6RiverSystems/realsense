/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#include <srsnode_midbrain/Reflexes.hpp>
#include <srslib_framework/math/PoseMath.hpp>
#include <srslib_framework/ros/message/PoseMessageFactory.hpp>
#include <srslib_framework/ros/message/SolutionMessageFactory.hpp>
#include <boost/geometry/algorithms/union.hpp>

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

std::ostream& operator<<( std::ostream& os, const Ring& polygon )
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

ObstacleDetector::ObstacleDetector( double footprintWidth, double footprintHeight ) :
	m_obstacleDetectedCallback( ),
	m_pose( Pose<>::INVALID ),
	m_footprintPolygon( ),
	m_posePolygon( ),
	m_solution( ),
	m_dangerZone( ),
	m_footprintWidth( footprintWidth ),
	m_footprintLength( footprintHeight),
	m_linearVelocity( 0.0f ),
	m_angularVelocity( 0.0f ),
	m_threshold( 0 )
{

}

ObstacleDetector::~ObstacleDetector( )
{

}

void ObstacleDetector::SetDetectionCallback( ObstacleDetectedFn obstacleDetectedCallback )
{
	m_obstacleDetectedCallback = obstacleDetectedCallback;
}

void ObstacleDetector::SetVelocity( double linearVelocity, double angularVelocity )
{
	m_linearVelocity = linearVelocity;

	m_angularVelocity = angularVelocity;
}

void ObstacleDetector::SetPose( const srslib_framework::MsgPose::ConstPtr& pose )
{
	m_pose = PoseMessageFactory::msg2Pose(*pose);

	m_footprintPolygon = Polygon( );
	m_posePolygon = Polygon( );

	AddPoseToPolygon( m_pose, m_footprintPolygon, m_footprintWidth, m_footprintLength );
	AddPoseToPolygon( m_pose, m_posePolygon, m_footprintWidth, m_footprintWidth );
}

void ObstacleDetector::SetSolution( const srslib_framework::MsgSolution::ConstPtr& solution )
{
	m_solution = SolutionMessageFactory::msg2Solution(*solution);

	UpdateDangerZone( );
}

void ObstacleDetector::SetThreshold( uint32_t threshold )
{
	m_threshold = threshold;
}

void ObstacleDetector::ProcessScan( const sensor_msgs::LaserScan::ConstPtr& scan )
{
	if( m_pose.isValid( ) )
	{
		uint32_t numberOfObstacles = 0;

		uint32_t numberOfScans = scan->ranges.size( );

		double safeDistance = GetSafeDistance( m_linearVelocity, m_angularVelocity );

		std::string strObstaclePoints;
		std::string strValidPoints;
		char pszPoints[255] = { '\0' };

		// Translate from /base_footprint => /laser_frame (around the rotated robot)
		Pose<> laserFramePoint = PoseMath::translate( m_pose, 0.489, 0.0 );

		for( int x = 0; x < numberOfScans; x++ )
		{
			double angle = ((double)x * scan->angle_increment) + scan->angle_min;

			double scanDistance = scan->ranges[x];

			if( std::isfinite( scanDistance ) )
			{
				//               fY
				//             ------
				//             \    |
				//              \   |
				// scan distance \  | fX (Distance from chuck)
				//                \0|
				//                 \|

				// Rotate around the laser angle
				Pose<> laserScanPose = PoseMath::rotate( laserFramePoint, angle );

				// Translate by the scan distance
				laserScanPose = PoseMath::translate( laserScanPose, scanDistance, 0.0 );

				// Convert to boost geometry point
				Point point( laserScanPose.x, laserScanPose.y );

				// Calculate the distance from the robot polygon and the point (both in map space)
				double distanceFromChuck = bg::distance( m_footprintPolygon, point );

				sprintf( pszPoints, "(%.2f => %.2f, %.2f => %.2f) ", scanDistance, point.x, point.y, distanceFromChuck );

				if( boost::geometry::within( point, m_dangerZone ) )
				{
					if( distanceFromChuck < safeDistance  )
					{
						strObstaclePoints += pszPoints;

						numberOfObstacles++;
					}
					else if( distanceFromChuck < safeDistance* 2 )
					{
						strValidPoints += pszPoints;
					}
				}
			}
		}

		// Allow some number of spurious points that can make it through our filtering
		if( numberOfObstacles > m_threshold )
		{
			ROS_INFO_NAMED( "obstacle_detection", "Scans: %d, linear vel: %0.2f, angular vel: %0.2f, \
				safeDistance: %.02f, Obstacles detected: %d, points: %s", numberOfScans,
				m_linearVelocity, m_angularVelocity, safeDistance, numberOfObstacles, strObstaclePoints.c_str( ) );

			if( m_obstacleDetectedCallback )
			{
				m_obstacleDetectedCallback( );
			}
		}
		else
		{
			if( strValidPoints.size( ) )
			{
				ROS_DEBUG_THROTTLE_NAMED( 1.0, "obstacle_detection", "Scans: %d, linear vel: %0.2f, angular vel: %0.2f, \
					safeDistance: %.02f, Objects detected: %d, points: %s", numberOfScans, m_linearVelocity,
					m_angularVelocity, safeDistance, numberOfObstacles, strValidPoints.c_str( ) );
			}
		}
	}
}

double ObstacleDetector::GetSafeDistance( double linearVelocity, double angularVelocity ) const
{
	double safeDistance = 0.0f;

	if( linearVelocity > 0.1 )
	{
		// Calculated from actual measurements and polynomial regression fitting
		// https://docs.google.com/a/6river.com/spreadsheets/d/1wUPgpESlp-MVbnMGCmFDGH2/2qyO39D8kzvi7IPZ1Q1c/edit?usp=sharing
		// http://www.xuru.org/rt/PR.asp#CopyPaste
		double calculatedSafeDistance = ( 0.3518981019f * pow( linearVelocity, 2 ) - 0.03806193806f * linearVelocity + 0.01804195804f ) * 2.0;

		safeDistance = std::max( calculatedSafeDistance, 0.75 );

		ROS_DEBUG_THROTTLE_NAMED( 1.0f, "obstacle_detection", "calculatedSafeDistance: %0.2f, safeDistance: %0.2f", calculatedSafeDistance, safeDistance ); 
	}

	return safeDistance;
}

double ObstacleDetector::GetFootprintWidth( ) const
{
	return m_footprintWidth;
}

double ObstacleDetector::GetFootprintLength( ) const
{
	return m_footprintLength;
}

Ring ObstacleDetector::GetDangerZone( ) const
{
	return m_dangerZone.outer( );
}

void ObstacleDetector::AddPoseToPolygon( const Pose<>& pose, Polygon& polygon, double width, double length ) const
{
	std::vector<Pose<>> posePoly = PoseMath::pose2polygon(pose, 0.0, 0.0, width, length );

	Ring footprintPoints;

	for( auto point : posePoly )
	{
		footprintPoints.push_back( { point.x, point.y } );
	}

	Polygon footprintPolygon;
	boost::geometry::assign_points( footprintPolygon, footprintPoints );

	std::vector<Polygon> combinedZone;

	// Normalize the polygons (if not the union function will not work properly)
	bg::correct( footprintPolygon );
	bg::correct( polygon );

	// Create the combined polygon by creating a union of the pose polygon and the input polygon
	bg::union_( footprintPolygon, polygon, combinedZone );

	assert( combinedZone.size( ) );

	if( combinedZone.size( ) )
	{
		polygon = combinedZone[0];
	}
}

void ObstacleDetector::UpdateDangerZone( )
{
	Polygon dangerZone;

	if( m_solution.size( ) )
	{
		// Build the danger zone from the solution
		for( auto solutionItem : m_solution )
		{
			AddPoseToPolygon( solutionItem.toPose, dangerZone, m_footprintWidth, m_footprintWidth );
		}
	}

	m_dangerZone = dangerZone;
}

} /* namespace srs */
