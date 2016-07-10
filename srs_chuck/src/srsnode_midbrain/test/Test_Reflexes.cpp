/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#include <gtest/gtest.h>
#include <boost/filesystem.hpp>
#include <Reflexes.hpp>
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

namespace srs {

namespace bg = boost::geometry;

typedef bg::model::point<double, 2, bg::cs::cartesian> point_t;
typedef bg::model::segment<point_t> segment_t;

class TestSubscriber
{
private:
	ros::Subscriber m_subscriber;

	bool& m_finished;

public:

	TestSubscriber( ros::NodeHandle& nodeHandle, bool& finished ) :
		receivedMessage( false ),
		m_finished( finished )
	{
		m_subscriber = nodeHandle.subscribe( "/ll_event", 1, &TestSubscriber::callback, this );
	}

	void callback( const std_msgs::String::ConstPtr &newMessage )
	{
		receivedMessage = true;
		message = newMessage;

		m_finished = true;
	}

	bool receivedMessage;
	std_msgs::StringConstPtr message;
};

class ReflexesTest : public ::testing::Test
{
public:

	ros::NodeHandle m_nodeHandle;

	Reflexes m_reflexes;

	bool m_finished;

	TestSubscriber subscriber;

	sensor_msgs::LaserScan::Ptr m_laserScan;


public:

	ReflexesTest( ) :
		m_reflexes( ),
		m_finished( false ),
		subscriber( m_nodeHandle, m_finished )

	{
		m_reflexes.Initialize( m_nodeHandle );
	}

	~ReflexesTest( )
	{

	}

	void SetUp( )
	{
		m_laserScan = CreateLaserScan( );
	}

	void TearDown( )
	{
		m_laserScan.reset( );
	}

	sensor_msgs::LaserScan::Ptr CreateLaserScan( )
	{
		sensor_msgs::LaserScan::Ptr laserScan( new sensor_msgs::LaserScan( ) );

		laserScan->angle_min = -1.57079994678;
		laserScan->angle_max = 1.57079994678;
		laserScan->angle_increment = 0.00800000037998;
		laserScan->time_increment = 0.0;
		laserScan->scan_time = 0.333299994469;
		laserScan->range_min = 0.0;
		laserScan->range_max = 8.0;
		uint32_t numberOfScans = std::ceil((laserScan->angle_max - laserScan->angle_min) / laserScan->angle_increment);
		laserScan->ranges.assign( numberOfScans, std::numeric_limits<double>::infinity( ) );

		return laserScan;
	}

	void AddObject( )
	{
		sensor_msgs::LaserScan::Ptr laserScan = CreateLaserScan( );

		uint32_t numberOfScans = m_laserScan->ranges.size( );

		double lineDistance = 100000.0f;

		segment_t objectLine( point_t( 0.3, -0.3 ), point_t( 0.3, 0.3 ) );

		for( int x = 0; x < numberOfScans; x++ )
		{
			double angle = ((double)x * m_laserScan->angle_increment) + m_laserScan->angle_min;

			// Create a very large line based on the laser scan
			segment_t laserLine( point_t( 0.0, 0.0), point_t( lineDistance * cos( angle ), lineDistance * sin( angle ) ) );

			std::vector<point_t> output;
			boost::geometry::intersection( objectLine, laserLine, output );

//			ROS_DEBUG_NAMED( "reflexes", "Laser Line: %.2f, %.2f - %.2f, %.2f, Object Line: %.2f, %.2f = %.2f, %.2f",
//				laserLine.first.get<0>( ), laserLine.first.get<1>( ), laserLine.second.get<0>( ), laserLine.second.get<1>( ),
//				objectLine.first.get<0>( ), objectLine.first.get<1>( ), objectLine.second.get<0>( ), objectLine.second.get<1>( ) );

			if( output.size( ) )
			{
				m_laserScan->ranges[x] = boost::geometry::distance( point_t( 0.0, 0.0), output[0]);

//				ROS_DEBUG_NAMED( "reflexes", "Angle %.02f: Intersection: %.2f, %.2f, Distance: %0.2f",
//					angle,
//					output[0].get<0>( ), output[0].get<1>( ),
//					laserScan->ranges[x]);

			}
		}
	}

	void SetPauseState( bool paused )
	{
		srslib_framework::MsgOperationalState::Ptr operationalState( new srslib_framework::MsgOperationalState( ) );

		operationalState->pause = paused;

		m_reflexes.OnOperationalStateChanged( operationalState );
	}

	void SetVelocity( double linear, double angular )
	{
		geometry_msgs::Twist::Ptr twist( new geometry_msgs::Twist( ) );

		twist->linear.x = linear;
		twist->angular.z = angular;

		m_reflexes.OnChangeVelocity( twist );
	}

	void WaitForMessage( )
	{
		boost::timer timer;

		while( !m_finished )
		{
		    ros::spinOnce( );

		    if( timer.elapsed( ) > 1.0f )
		    {
		    	break;
		    }
		}
	}
};

TEST_F( ReflexesTest, TestNoObstacleNoVelocity )
{
	m_reflexes.OnLaserScan( m_laserScan );

	EXPECT_FALSE( subscriber.receivedMessage ); // This may or may not be true
}

TEST_F( ReflexesTest, TestObstacleNoVelocity )
{
	AddObject( );

	m_reflexes.OnLaserScan( m_laserScan );

	WaitForMessage( );

	EXPECT_TRUE( subscriber.receivedMessage ); // This may or may not be true
}

}  // namespace

int main(int argc, char **argv)
{
	testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "reflexes_unit_test");
	ros::NodeHandle nh;
	return RUN_ALL_TESTS();
}
