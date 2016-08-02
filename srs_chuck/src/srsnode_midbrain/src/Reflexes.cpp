/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#include <srsnode_midbrain/Reflexes.hpp>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PolygonStamped.h>
#include <tgmath.h>
#include <boost/range/adaptor/reversed.hpp>

namespace srs
{

Reflexes::Reflexes( ros::NodeHandle& nodeHandle ) :
	m_nodeHandle( nodeHandle ),
	m_enableIrDetection( false ),
	m_enableDepthDetection( false ),
	m_sendHardStop( false ),
	m_operationalState( ),
	m_obstacleDetector( ROBOT_WIDTH / 2.0f ),
	m_operationalStateSubscriber( ),
	m_irScanSubscriber( ),
	m_depthScanSubscriber( ),
	m_velocitySubscriber( ),
	m_commandPublisher( )
{
	bool obstacleDetected = false;

	m_obstacleDetector.SetDetectionCallback( std::bind( &Reflexes::OnObstacleDetected, this ) );

	m_configServer.setCallback( boost::bind( &Reflexes::onConfigChange, this, _1, _2 ) );

	CreateSubscribers( );

	CreatePublishers( );
}

Reflexes::~Reflexes( )
{

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Configuration Options
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Reflexes::Enable( bool enableDepthDetection )
{
	if( m_enableDepthDetection != enableDepthDetection )
	{
		if( enableDepthDetection )
		{
			ROS_INFO_NAMED( "obstacle_detection", "Reflexes: Enabling Depth Detection" );

			m_depthScanSubscriber = m_nodeHandle.subscribe<sensor_msgs::LaserScan>(
					DEPTH_SCAN_TOPIC, 10, std::bind( &Reflexes::OnLaserScan, this, std::placeholders::_1, false ) );
		}
		else
		{
			ROS_INFO_NAMED( "obstacle_detection", "Disabling Depth Detection" );

			m_depthScanSubscriber.shutdown( );
		}

		m_enableDepthDetection = enableDepthDetection;
	}
}

void Reflexes::SetObjectThreshold( uint32_t depthThreshold )
{
	m_obstacleDetector.SetThreshold( depthThreshold );
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Topic Callbacks
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Reflexes::OnOperationalStateChanged( const srslib_framework::MsgOperationalState::ConstPtr& operationalState )
{
	ROS_INFO_STREAM_NAMED( "obstacle_detection", "Reflexes: OnOperationalStateChanged: " << operationalState->pause );

	m_operationalState = *operationalState;
}

void Reflexes::OnChangeVelocity( const geometry_msgs::TwistStamped::ConstPtr& velocity )
{
	m_obstacleDetector.SetVelocity( velocity->twist.linear.x, velocity->twist.angular.z );
}

void Reflexes::OnLaserScan( const sensor_msgs::LaserScan::ConstPtr& scan, bool isIrScan )
{
	m_obstacleDetector.ProcessScan( scan, isIrScan );
}

void Reflexes::PublishDangerZone( ) const
{
	geometry_msgs::PolygonStamped messageLanding;

	messageLanding.header.frame_id = "laser_frame";
	messageLanding.header.stamp = ros::Time::now( );

	std::vector<geometry_msgs::Point32> polygon;

	Polygon dangerZone = m_obstacleDetector.GetDangerZone( );

	for( const Point& point : dangerZone )
	{
		geometry_msgs::Point32 corner;
		corner.x = point.x;
		corner.y = point.y;
		corner.z = 0.0;

		polygon.push_back( corner );
	}

	messageLanding.polygon.points = polygon;

	m_dangerZonePublisher.publish( messageLanding );
}

void Reflexes::onConfigChange(srsnode_midbrain::ReflexesConfig& config, uint32_t level)
{
	ROS_INFO_NAMED( "obstacle_detection", "Reflexes: Midbrain config changed: hardStop: %d, depth scan: %d, depth threshold: %d",
		config.depth_threshold, config.enable_hard_stop, config.depth_threshold );

	SetObjectThreshold( config.depth_threshold );

	Enable( config.enable_depth_scan );

	m_sendHardStop = config.enable_hard_stop;
}

void Reflexes::OnObstacleDetected( )
{
	ROS_INFO_NAMED( "obstacle_detection", "Reflexes: OnObstacleDetected: paused: %d, hardStop: %d",
		m_operationalState.pause, m_sendHardStop );

	// Only send a hard stop if we are paused
	if( !m_operationalState.pause &&
		!m_operationalState.hardStop &&
		m_sendHardStop )
	{
		ROS_INFO_STREAM_NAMED( "obstacle_detection", "Reflexes: OnObstacleDetected: Sending STOP" );

		// Send the hard stop
		std_msgs::String msg;
		msg.data = "STOP";

		m_commandPublisher.publish( msg );
	}
}

void Reflexes::CreateSubscribers( )
{
	m_operationalStateSubscriber = m_nodeHandle.subscribe<srslib_framework::MsgOperationalState>(
		OPERATIONAL_STATE_TOPIC, 10, std::bind( &Reflexes::OnOperationalStateChanged, this, std::placeholders::_1 ) );

	m_velocitySubscriber = m_nodeHandle.subscribe<geometry_msgs::TwistStamped>( ODOMETRY_TOPIC, 1,
	    std::bind( &Reflexes::OnChangeVelocity, this, std::placeholders::_1 ) );
}

void Reflexes::CreatePublishers( )
{
	m_commandPublisher = m_nodeHandle.advertise<std_msgs::String>( COMMAND_TOPIC, 1, true );

	m_dangerZonePublisher = m_nodeHandle.advertise<geometry_msgs::PolygonStamped>( DANGER_ZONE_TOPIC, 1 );
}

} /* namespace srs */
