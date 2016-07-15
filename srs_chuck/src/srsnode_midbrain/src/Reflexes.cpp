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

Reflexes::Reflexes( ros::NodeHandle& nodeHandle ) :
	m_nodeHandle( nodeHandle ),
	m_objectThreshold( 0 ),
	m_maxLinearVelocity( 0.5f ),
	m_maxAngularVelocity( 0.7f ),
	m_chuckWidth_( 0.619125 ),
	m_chuckHalfWidth( m_chuckWidth_ / 2.0f ),
	m_yPaddedOffset( m_chuckHalfWidth * 1.2 ),
	m_minSafeDistance( 0.35f ),
	m_maxSafeDistance( 0.5f ),
	m_operationalState( ),
	m_velocity( ),
	m_operationalStateSubscriber( ),
	m_laserScanSubscriber( ),
	m_velocitySubscriber( ),
	m_commandPublisher( )
{
	ROS_INFO( "Reflex: half width: %f, yOffset: %f, safeDistance: %f/%f",
		m_chuckHalfWidth, m_yPaddedOffset, m_minSafeDistance, m_maxSafeDistance );

	Enable( true );
}

Reflexes::~Reflexes( )
{

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Configuration Options
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Reflexes::Enable( bool enable )
{
	if( enable )
	{
		CreateSubscribers( );

		CreatePublishers( );
	}
	else
	{
		DestroySubscribers( );

		DestroyPublishers( );
	}
}

void Reflexes::SetObjectThreshold( uint32_t objectThreshold )
{
	m_objectThreshold = objectThreshold;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Topic Callbacks
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Reflexes::OnOperationalStateChanged( const srslib_framework::MsgOperationalState::ConstPtr& operationalState )
{
	m_operationalState = *operationalState;
}

void Reflexes::OnChangeVelocity( const geometry_msgs::Twist::ConstPtr& velocity )
{
	m_velocity = *velocity;
}

void Reflexes::OnLaserScan( const sensor_msgs::LaserScan::ConstPtr& scan )
{
	uint32_t numberOfObstacles = 0;

	uint32_t numberOfScans = scan->ranges.size( );

	double safetyThrottle = m_velocity.linear.x / m_maxLinearVelocity;

	double safeDistance = m_maxSafeDistance * safetyThrottle;

	safeDistance = std::min( safeDistance, m_maxSafeDistance );
	safeDistance = std::max( safeDistance, m_minSafeDistance );

	std::string strPoints;
	char point[255] = { '\0' };
	sprintf( point, "Safe (throttle: %.2f): %.2f: ", safeDistance, safetyThrottle );
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
				if( fabs( yOffset ) < m_yPaddedOffset )
				{
					numberOfObstacles++;
				}
			}
		}
	}

	ROS_DEBUG_THROTTLE_NAMED( 0.3, "reflexes", "linear vel: %0.2f, safetyThrottle: %.02f, safeDistance: %.02f, Obstacles detected: %d, points: %s",
		m_velocity.linear.x, safetyThrottle, safeDistance, numberOfObstacles, strPoints.c_str( ) );

	if( numberOfObstacles > m_objectThreshold )
	{
		// Only send a hard stop if we are paused
		if( !m_operationalState.pause )
		{
			// Send the hard stop
			std_msgs::String msg;
			msg.data = "STOP";

			m_commandPublisher.publish( msg );
		}
	}
*/
}

void Reflexes::onConfigChange(srsnode_midbrain::ReflexesConfig& config, uint32_t level)
{
	SetObjectThreshold( config.object_threshold );

	Enable( config.enable_obstacle_detection );

	m_sendHardStop = config.enable_hard_stop;
}

void Reflexes::CreateSubscribers( )
{
	m_operationalStateSubscriber = m_nodeHandle.subscribe<srslib_framework::MsgOperationalState>(
		OPERATIONAL_STATE_TOPIC, 10, std::bind( &Reflexes::OnOperationalStateChanged, this, std::placeholders::_1 ) );

	m_laserScanSubscriber = m_nodeHandle.subscribe<sensor_msgs::LaserScan>(
		SCAN_TOPIC, 10, std::bind( &Reflexes::OnLaserScan, this, std::placeholders::_1 ) );

	m_velocitySubscriber = m_nodeHandle.subscribe<geometry_msgs::Twist>( VELOCITY_TOPIC, 1,
	    std::bind( &Reflexes::OnChangeVelocity, this, std::placeholders::_1 ) );
}

void Reflexes::DestroySubscribers( )
{
	m_operationalStateSubscriber.shutdown( );

	m_laserScanSubscriber.shutdown( );

	m_velocitySubscriber.shutdown( );
}

void Reflexes::CreatePublishers( )
{
	m_commandPublisher = m_nodeHandle.advertise<std_msgs::String>( EVENT_TOPIC, 1, true );
}

void Reflexes::DestroyPublishers( )
{
	m_commandPublisher.shutdown( );
}

} /* namespace srs */
