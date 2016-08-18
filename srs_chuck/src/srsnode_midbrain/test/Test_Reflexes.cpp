/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#include <gtest/gtest.h>
#include <boost/filesystem.hpp>
#include <srsnode_midbrain/ObstacleDetector.hpp>
#include <srslib_framework/MsgOperationalState.h>
#include <std_msgs/String.h>
#include <boost/timer.hpp>
#include <boost/geometry/algorithms/intersection.hpp>

namespace srs {

struct stopping_distance_t
{
	double velocity;
	double distance;
};

class ReflexesTest : public ::testing::Test
{
public:

	double m_robotWidth;

	double m_robotDepth;

	double m_lineWidth;

	ObstacleDetector m_detector;

	std::vector<double> m_vecVelocities;
	std::vector<double> m_vecOffsets;
	std::vector<double> m_vecDistances;

public:

	ReflexesTest( ) :
		m_robotWidth( 0.619125 ),
		m_robotDepth( 0.9772396 ),
		m_lineWidth( 0.3 ),
		m_detector( m_robotWidth, m_robotDepth ),
		m_vecVelocities( ),
		m_vecOffsets( ),
		m_vecDistances( )
	{
		m_vecVelocities.push_back( 0.00 );
		m_vecVelocities.push_back( 0.20 );
		m_vecVelocities.push_back( 0.40 );
		m_vecVelocities.push_back( 0.60 );
		m_vecVelocities.push_back( 0.80 );
		m_vecVelocities.push_back( 1.00 );
		m_vecVelocities.push_back( 1.20 );
		m_vecVelocities.push_back( 1.40 );
		m_vecVelocities.push_back( 1.60 );
		m_vecVelocities.push_back( 1.80 );
		m_vecVelocities.push_back( 2.00 );
		m_vecVelocities.push_back( 2.20 );
		m_vecVelocities.push_back( 2.40 );
		m_vecVelocities.push_back( 2.60 );

		m_vecOffsets.push_back( 0 );
		m_vecOffsets.push_back( -(m_robotWidth/2.0 + m_lineWidth + 0.001 ) );
		m_vecOffsets.push_back( m_robotWidth/2.0 + 0.001 );

		m_vecDistances.push_back( 0.02 );
		m_vecDistances.push_back( 0.07 );
		m_vecDistances.push_back( 0.12 );
		m_vecDistances.push_back( 0.34 );
		m_vecDistances.push_back( 0.47 );
		m_vecDistances.push_back( 0.65 );
		m_vecDistances.push_back( 0.86 );
		m_vecDistances.push_back( 1.08  );
		m_vecDistances.push_back( 1.34 );
		m_vecDistances.push_back( 1.62 );
		m_vecDistances.push_back( 2.05 );
		m_vecDistances.push_back( 2.24 );
		m_vecDistances.push_back( 2.5 );
		m_vecDistances.push_back( 3.0 );
		m_vecDistances.push_back( 5.0 );
		m_vecDistances.push_back( 10.0 );
		m_vecDistances.push_back( 100.0 );
		m_vecDistances.push_back( 1000.0 );
	}

	~ReflexesTest( )
	{

	}

	void SetUp( )
	{

	}

	void TearDown( )
	{

	}

	sensor_msgs::LaserScan::Ptr CreateLaserScan( )
	{
		sensor_msgs::LaserScan::Ptr laserScan( new sensor_msgs::LaserScan( ) );

		laserScan->angle_min = -M_PI / 2.0f;
		laserScan->angle_max = M_PI / 2.0f;
		laserScan->angle_increment = M_PI / 360.0;
		laserScan->time_increment = 0.0;
		laserScan->scan_time = 1.0f / 30.0f; // 30 Hz
		laserScan->range_min = 0.0;
		laserScan->range_max = 8.0;
		uint32_t numberOfScans = std::ceil((laserScan->angle_max - laserScan->angle_min) / laserScan->angle_increment);
		laserScan->ranges.assign( numberOfScans, std::numeric_limits<double>::infinity( ) );

		return laserScan;
	}

	void AddObject( sensor_msgs::LaserScan::Ptr laserScan, Segment lineSegment )
	{
		uint32_t numberOfScans = laserScan->ranges.size( );

		double lineDistance = 100000.0f;

		double numberOfLaserPoints = 0;

		uint32_t lastScan = 0;

		for( int i = 0; i < numberOfScans; i++ )
		{
			double angle = ((double)i * laserScan->angle_increment) + laserScan->angle_min;

			double fStartX = -lineDistance * cos( angle );
			double fStartY = -lineDistance * sin( angle );
			double fEndX = lineDistance * cos( angle );
			double fEndY = lineDistance * sin( angle );

			// Create a very large line based on the laser scan
			Segment laserLine( Point( fStartX, fStartY ), Point( fEndX, fEndY ) );

			std::vector<Point> output;
			bg::intersection( lineSegment, laserLine, output );

			if( output.size( ) )
			{
				numberOfLaserPoints++;

				laserScan->ranges[i] = bg::distance( Point( 0.0, 0.0 ), output[0] );

//				ROS_DEBUG_STREAM_NAMED( "obstacle_detection", "Laser Intersects Object (" <<  output.size( ) << ") " << lineSegment <<
//					" @ Angle: " << angle * 180.0f / M_PI << " Distance: " <<  laserScan->ranges[i] );
			}
		}
	}

	Segment CreateSegment( double distance, double offset, double length )
	{
		return Segment( Point( distance, offset - ( length / 2.0f) ),
			Point( distance, offset + (length / 2.0f) ) );
	}
};

TEST_F( ReflexesTest, TestNoObstacleNoVelocity )
{
	sensor_msgs::LaserScan::Ptr laserScan = CreateLaserScan( );

	bool obstacleDetected = false;

	m_detector.SetDetectionCallback( [&]() {
		obstacleDetected = true;
	});

	m_detector.ProcessScan( laserScan );

	EXPECT_FALSE( obstacleDetected );
}

TEST_F( ReflexesTest, TestObstacleNoVelocity )
{
	for( auto distance : m_vecDistances )
	{
		Segment lineSegment = CreateSegment( distance, 0.0, m_lineWidth );

		sensor_msgs::LaserScan::Ptr laserScan = CreateLaserScan( );

		AddObject( laserScan, lineSegment );

		bool obstacleDetected = false;

		m_detector.SetDetectionCallback( [&]() {
			obstacleDetected = true;
		});

		m_detector.ProcessScan( laserScan );

		EXPECT_FALSE( obstacleDetected );
	}
}

TEST_F( ReflexesTest, TestMovingWithObstacle )
{
//	boost::geometry::strategy::transform::scale_transformer<Point, Point> scale( 10.0f, 10.0f );
//
//	double footprintWidth = m_detector.GetFootprintWidth( );
//
//	int index = 0;
//
//	for( auto offset : m_vecOffsets )
//	{
//		for( auto velocity : m_vecVelocities )
//		{
//			m_detector.SetVelocity( velocity, 0.0f );
//
//			for( auto distance : m_vecDistances )
//			{
//				Segment lineSegment = CreateSegment( distance, offset, m_lineWidth );
//
//				sensor_msgs::LaserScan::Ptr laserScan = CreateLaserScan( );
//
//				AddObject( laserScan, lineSegment );
//
//				bool obstacleDetected = false;
//
//				m_detector.SetDetectionCallback( [&]() {
//					obstacleDetected = true;
//				});
//
//				m_detector.ProcessScan( laserScan );
//
//				using Ring = std::vector<Point>;
//
//				double safeDistance = m_detector.GetSafeDistance( velocity, 0.0 );
//
//				Ring dangerZone({
//					{ 0.0, -footprintWidth },
//					{ safeDistance, -footprintWidth },
//					{ safeDistance, footprintWidth },
//					{ 0.0, footprintWidth }
//				});
//
//				Ring lineRing( {
//					{ lineSegment.first },
//					{ Point( lineSegment.first.x + 0.2, lineSegment.first.y ) },
//					{ Point( lineSegment.second.x + 0.2, lineSegment.second.y ) },
//					{ lineSegment.second },
//				} );
//
//
//				boost::geometry::correct( dangerZone );
//				boost::geometry::correct( lineRing );
//
//	//			ROS_DEBUG_STREAM_NAMED( "obstacle_detection", "Danger Zone: " << dangerZone );
//	//			ROS_DEBUG_STREAM_NAMED( "obstacle_detection", "Object: " << lineRing );
//
//				std::deque<Ring> intersection;
//				bg::intersection( dangerZone, lineRing, intersection );
//
//				bool intersects = bg::intersects( dangerZone, lineRing );
//
//				char pszFile[1024] = { '\0' };
//				sprintf( pszFile, "/tmp/test-%d-speed-%.1f-safe_distance-%.2f-obstacle_distance-%.2f.svg", index++,
//					velocity, safeDistance, lineSegment.first.x);
//
//				std::ofstream svg( pszFile );
//				boost::geometry::svg_mapper<Point> mapper( svg, 400, 400 );
//
//				Ring scaledDangerZone;
//				boost::geometry::transform( dangerZone, scaledDangerZone, scale );
//				mapper.add( scaledDangerZone );
//				mapper.map( scaledDangerZone, "fill-opacity:0.3;fill:rgb(0,212,0);stroke:none" );
//
//				Ring scaledObstacle;
//				boost::geometry::transform( lineRing, scaledObstacle, scale );
//				mapper.add( scaledObstacle );
//				mapper.map( scaledObstacle, "fill-opacity:0.3;fill:rgb(0,0,212);stroke:none" );
//
//				for( auto polygon : intersection )
//				{
//	//				ROS_DEBUG_STREAM_NAMED( "obstacle_detection", "Intersection: " << polygon );
//
//					Ring scaledIntersection;
//					boost::geometry::transform( polygon, scaledIntersection, scale );
//					mapper.add( scaledIntersection );
//					mapper.map( scaledIntersection, "fill-opacity:1;fill:rgb(212,0,0);stroke:none" );
//				}
//
//				// Add laser scan points to debug svg file
//				uint32_t numberOfScans = laserScan->ranges.size( );
//				for( int x = 0; x < numberOfScans; x++ )
//				{
//					double angle = ((double)x * laserScan->angle_increment) + laserScan->angle_min;
//
//					double distance = laserScan->ranges[x];
//					if( distance != std::numeric_limits<double>::infinity( ) )
//					{
//						double x = distance * cos( angle );
//						double y = distance * sin( angle );
//
//						constexpr double width = 0.01f;
//
//						Ring laserPoint({
//							{ x - width, y - width },
//							{ x - width, y + width },
//							{ x + width, y + width },
//							{ x + width, y - width },
//							{ x - width, y - width }
//						});
//						Ring scaledLaserPoint;
//						boost::geometry::transform( laserPoint, scaledLaserPoint, scale );
//						mapper.add( scaledLaserPoint );
//						mapper.map( scaledLaserPoint, "fill-opacity:1;fill:rgb(255,20,147);stroke:none" );
//					}
//				}
//
//				ROS_DEBUG_STREAM_NAMED( "obstacle_detection", "Object @ distance: " << lineSegment.first.x << " with footprint " <<
//					footprintWidth << (intersects ? " is" : " is not") << " going to collide with chuck [safe distance: " <<
//					safeDistance << "] (" << pszFile << ")" );
//
//				if( intersects )
//				{
//					EXPECT_TRUE( obstacleDetected );
//				}
//				else
//				{
//					EXPECT_FALSE( obstacleDetected );
//				}
//			}
//		}
//	}
}

}  // namespace

int main(int argc, char **argv)
{
	testing::InitGoogleTest(&argc, argv);

	ros::Time::init( );

	return RUN_ALL_TESTS();
}
