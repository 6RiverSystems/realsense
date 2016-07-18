/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#include <gtest/gtest.h>
#include <boost/filesystem.hpp>
#include <srsnode_midbrain/ObstacleDetector.hpp>
#include <std_msgs/String.h>
#include <boost/timer.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/core/cs.hpp>
#include <boost/geometry/io/io.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/geometries/register/ring.hpp>
#include <fstream>
#include <iostream>

namespace bg = boost::geometry;

class Point {
public:
    double _x, _y;
    Point():_x(),_y(){}
    Point(double x, double y):_x(x),_y(y){}

    Point& operator+=(const Point& rhs)
	{
    	return *this;
	}

    friend Point operator+(Point lhs, const Point& rhs)
	{
		lhs += rhs;
		return lhs;
	}

	friend std::ostream& operator<<( std::ostream& os, const Point& point )
    {
    	os << point._x << ", " << point._y;
    }

};

typedef bg::model::segment<Point> segment_t;
typedef bg::model::ring<Point> ring_t;
typedef std::vector<Point> Polygon;

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

		os << point._x << ", " << point._y;

		first = false;
	}

	os << ")";

	return os;
}

std::ostream& operator<<( std::ostream& os, const segment_t& segment )
{
	os << segment.first._x << ", " << segment.first._y;
	os << " - ";
	os << segment.second._x << ", " << segment.second._y;

	return os;
}

BOOST_GEOMETRY_REGISTER_POINT_2D(Point, double, bg::cs::cartesian, _x, _y)
BOOST_GEOMETRY_REGISTER_RING(Polygon)

namespace srs {

struct stopping_distance_t
{
	double velocity;
	double distance;
};

class ReflexesTest : public ::testing::Test
{
public:

	ObstacleDetector m_detector;

	std::vector<double> m_vecVelocities;

public:

	ReflexesTest( ) :
		m_detector( 0.619125 / 2.0 ),
		m_vecVelocities( )
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

	void AddObject( sensor_msgs::LaserScan::Ptr laserScan, segment_t lineSegment )
	{
		uint32_t numberOfScans = laserScan->ranges.size( );

		double lineDistance = 100000.0f;

		double numberOfLaserPoints = 0;

		uint32_t lastScan = 0;

		for( int i = 0; i < numberOfScans; i++ )
		{
			double angle = ((double)i * laserScan->angle_increment) + laserScan->angle_min;

			double fX = lineDistance * cos( angle );
			double fY = lineDistance * sin( angle );

			// Create a very large line based on the laser scan
			segment_t laserLine( Point( 0.0, 0.0 ), Point( fX, fY ) );

			std::vector<Point> output;
			boost::geometry::intersection( lineSegment, laserLine, output );

			if( output.size( ) )
			{
				numberOfLaserPoints++;

				laserScan->ranges[i] = boost::geometry::distance( Point( 0.0, 0.0 ), output[0] );

//				ROS_DEBUG_STREAM_NAMED( "obstacle_detection", "Laser Intersects Object (" <<  output.size( ) << ") " << lineSegment <<
//					" @ Angle: " << angle * 180.0f / M_PI << " Distance: " <<  laserScan->ranges[i] );
			}
		}
	}

	void SetPauseState( bool paused )
	{
//		srslib_framework::MsgOperationalState::Ptr operationalState( new srslib_framework::MsgOperationalState( ) );
//
//		operationalState->pause = paused;

//		m_reflexes.OnOperationalStateChanged( operationalState );
	}

	segment_t CreateSegment( double distance, double offset, double length )
	{
		return segment_t( Point( distance, offset - ( length / 2.0f) ),
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
	std::vector<segment_t> vecSegments;

	vecSegments.push_back( CreateSegment( 10.0, 0.0, 20.0 ) );
	vecSegments.push_back( CreateSegment( 0.01, 0.0, 0.3 ) );
	vecSegments.push_back( CreateSegment( 0.3, 0.0, 0.3 ) );
	vecSegments.push_back( CreateSegment( 0.5, 0.0, 0.3 ) );
	vecSegments.push_back( CreateSegment( 1.0, 0.0, 0.3 ) );
	vecSegments.push_back( CreateSegment( 5.0, 0.0, 0.3 ) );
	vecSegments.push_back( CreateSegment( 10.0, 0.0, 0.3 ) );

	for( auto lineSegment : vecSegments )
	{
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

TEST_F( ReflexesTest, TestMovingWithSquareObstacle )
{
	boost::geometry::strategy::transform::scale_transformer<Point, Point> scale( 10.0f, 10.0f );

	double footprint = m_detector.GetFootprint( );

	int index = 0;

	for( auto velocity : m_vecVelocities )
	{
		m_detector.SetVelocity( velocity, 0.0f );

		std::vector<segment_t> vecSegments;

		vecSegments.push_back( CreateSegment( 0.01, 0.0, 0.3 ) );
		vecSegments.push_back( CreateSegment( 0.02, 0.0, 0.3 ) );
		vecSegments.push_back( CreateSegment( 0.07, 0.0, 0.3 ) );
		vecSegments.push_back( CreateSegment( 0.12, 0.0, 0.3 ) );
		vecSegments.push_back( CreateSegment( 0.34, 0.0, 0.3 ) );
		vecSegments.push_back( CreateSegment( 0.47, 0.0, 0.3 ) );
		vecSegments.push_back( CreateSegment( 0.65, 0.0, 0.3 ) );
		vecSegments.push_back( CreateSegment( 0.86, 0.0, 0.3 ) );
		vecSegments.push_back( CreateSegment( 1.08 , 0.0, 0.3 ) );
		vecSegments.push_back( CreateSegment( 1.34, 0.0, 0.3 ) );
		vecSegments.push_back( CreateSegment( 1.62, 0.0, 0.3 ) );
		vecSegments.push_back( CreateSegment( 2.05, 0.0, 0.3 ) );
		vecSegments.push_back( CreateSegment( 2.24, 0.0, 0.3 ) );
		vecSegments.push_back( CreateSegment( 2.5, 0.0, 0.3 ) );
		vecSegments.push_back( CreateSegment( 3.0, 0.0, 0.3 ) );
		vecSegments.push_back( CreateSegment( 5.0, 0.0, 0.3 ) );
		vecSegments.push_back( CreateSegment( 10.0, 0.0, 0.3 ) );
		vecSegments.push_back( CreateSegment( 100.0, 0.0, 0.3 ) );
		vecSegments.push_back( CreateSegment( 1000.0, 0.0, 0.3 ) );

		for( auto lineSegment : vecSegments )
		{
			sensor_msgs::LaserScan::Ptr laserScan = CreateLaserScan( );

			AddObject( laserScan, lineSegment );

			bool obstacleDetected = false;

			m_detector.SetDetectionCallback( [&]() {
				obstacleDetected = true;
			});

			m_detector.ProcessScan( laserScan );

			using Ring = std::vector<Point>;

			double safeDistance = m_detector.GetSafeDistance( velocity );

			Polygon dangerZone({
				{ 0.0, -footprint },
				{ safeDistance, -footprint },
				{ safeDistance, footprint },
				{ 0.0, footprint }
			});

			Polygon lineRing( {
				{ lineSegment.first },
				{ Point( lineSegment.first._x + 0.2, lineSegment.first._y ) },
				{ Point( lineSegment.second._x + 0.2, lineSegment.second._y ) },
				{ lineSegment.second },
			} );


			boost::geometry::correct( dangerZone );
			boost::geometry::correct( lineRing );

//			ROS_DEBUG_STREAM_NAMED( "obstacle_detection", "Danger Zone: " << dangerZone );
//			ROS_DEBUG_STREAM_NAMED( "obstacle_detection", "Object: " << lineRing );

			std::deque<Polygon> intersection;
			bg::intersection( dangerZone, lineRing, intersection );

			bool intersects = bg::intersects( dangerZone, lineRing );

			char pszFile[1024] = { '\0' };
			sprintf( pszFile, "/tmp/test-%d-speed-%.1f-safe_distance-%.2f-obstacle_distance-%.2f.svg", index++,
				velocity, safeDistance, lineSegment.first._x);

			std::ofstream svg( pszFile );
			boost::geometry::svg_mapper<Point> mapper( svg, 400, 400 );

			Polygon scaledDangerZone;
			boost::geometry::transform( dangerZone, scaledDangerZone, scale );
			mapper.add( scaledDangerZone );
			mapper.map( scaledDangerZone, "fill-opacity:0.3;fill:rgb(0,212,0);stroke:none" );

			Polygon scaledObstacle;
			boost::geometry::transform( lineRing, scaledObstacle, scale );
			mapper.add( scaledObstacle );
			mapper.map( scaledObstacle, "fill-opacity:0.3;fill:rgb(0,0,212);stroke:none" );

			for( auto polygon : intersection )
			{
//				ROS_DEBUG_STREAM_NAMED( "obstacle_detection", "Intersection: " << polygon );

				Polygon scaledIntersection;
				boost::geometry::transform( polygon, scaledIntersection, scale );
				mapper.add( scaledIntersection );
				mapper.map( scaledIntersection, "fill-opacity:1;fill:rgb(212,0,0);stroke:none" );
			}

			// Add laser scan points to debug svg file
			uint32_t numberOfScans = laserScan->ranges.size( );
			for( int x = 0; x < numberOfScans; x++ )
			{
				double angle = ((double)x * laserScan->angle_increment) + laserScan->angle_min;

				double distance = laserScan->ranges[x];
				if( distance != std::numeric_limits<double>::infinity( ) )
				{
					double x = distance * cos( angle );
					double y = distance * sin( angle );

					constexpr double width = 0.01f;

					Polygon laserPoint({
						{ x - width, y - width },
						{ x - width, y + width },
						{ x + width, y + width },
						{ x + width, y - width },
						{ x - width, y - width }
					});
					Polygon scaledLaserPoint;
					boost::geometry::transform( laserPoint, scaledLaserPoint, scale );
					mapper.add( scaledLaserPoint );
					mapper.map( scaledLaserPoint, "fill-opacity:1;fill:rgb(255,20,147);stroke:none" );
				}
			}

			ROS_DEBUG_STREAM_NAMED( "obstacle_detection", "Object @ distance: " << lineSegment.first._x << " with footprint " <<
				footprint << (intersects ? " is" : " is not") << " going to collide with chuck [safe distance: " <<
				safeDistance << "] (" << pszFile << ")" );

			if( intersects )
			{
				EXPECT_TRUE( obstacleDetected );
			}
			else
			{
				EXPECT_FALSE( obstacleDetected );
			}
		}
	}
}

}  // namespace

int main(int argc, char **argv)
{
	testing::InitGoogleTest(&argc, argv);

	ros::Time::init( );

	return RUN_ALL_TESTS();
}
