/*
 * MessageProcessor.cpp
 *
 *  Created on: Apr 7, 2016
 *      Author: dan
 */

#include "MessageProcessor.h"
#include "Messages.h"
#include "Helper.h"
#include "IO.h"

#include <chrono>
#include <boost/tokenizer.hpp>

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>

bool approximatively_equal(double x, double y, int ulp)
{
   return fabs(x-y) <= ulp*DBL_EPSILON*std::max(fabs(x), fabs(y));
}

namespace srs {

using namespace ros;

MessageProcessor::MessageProcessor( ros::NodeHandle& node, IO* pIO ) :
	m_bControllerFault( false ),
	m_dwLastOdomTime( 0 ),
	m_rosOdomTime( ),
	m_node( node ),
	m_pIO( pIO ),
	m_VelocitySubscriber( node.subscribe<geometry_msgs::Twist>( "/cmd_vel", 100,
		std::bind( &MessageProcessor::OnChangeVelocity, this, std::placeholders::_1 ) ) ),
	m_OdometryRawPublisher( node.advertise<geometry_msgs::TwistStamped>( "/sensors/odometry/raw", 1000 ) ),
	m_ConnectedPublisher( node.advertise<std_msgs::Bool>( "/brain_stem/connected", 1 ) ),
	m_llcmdSubscriber( node.subscribe<std_msgs::String>( "/cmd_ll", 1000,
			std::bind( &MessageProcessor::OnRosCallback, this, std::placeholders::_1 ) ) ),
	m_llEventPublisher( node.advertise<std_msgs::String>( "/ll_event", 50 ) )
{
	m_mapEntityButton[ENTITIES::TOTE0]		= "TOTE0";
	m_mapEntityButton[ENTITIES::TOTE1]		= "TOTE1";
	m_mapEntityButton[ENTITIES::TOTE2]		= "TOTE2";
	m_mapEntityButton[ENTITIES::TOTE3]		= "TOTE3";
	m_mapEntityButton[ENTITIES::TOTE4]		= "TOTE4";
	m_mapEntityButton[ENTITIES::TOTE5]		= "TOTE5";
	m_mapEntityButton[ENTITIES::TOTE6]		= "TOTE6";
	m_mapEntityButton[ENTITIES::TOTE7]		= "TOTE7";
	m_mapEntityButton[ENTITIES::ACTION]		= "ACTION";
	m_mapEntityButton[ENTITIES::PAUSE]		= "PAUSE";
	m_mapEntityButton[ENTITIES::TAIL_LEFT]	= "TAIL_LEFT";
	m_mapEntityButton[ENTITIES::TAIL_RIGHT]	= "TAIL_RIGHT";

	m_mapLedMode["OFF"] 					= LED_MODE::OFF;
	m_mapLedMode["ON"] 						= LED_MODE::ON;
	m_mapLedMode["GRAB"] 					= LED_MODE::GRAB;
	m_mapLedMode["PUT"] 					= LED_MODE::PUT;
	m_mapLedMode["BRAKE"] 					= LED_MODE::BRAKE;
	m_mapLedMode["TURN"] 					= LED_MODE::TURN;
	m_mapLedMode["SELECT"] 					= LED_MODE::SELECT;

	for( auto& kv : m_mapEntityButton )
	{
		m_mapButtonEntity[kv.second] = kv.first;
	}

	m_vecBridgeCallbacks["UI"] = { std::bind( &MessageProcessor::OnUI, this, std::placeholders::_1 ), 2 };
	m_vecBridgeCallbacks["STARTUP"] = { std::bind( &MessageProcessor::OnStartup, this, std::placeholders::_1 ), 0 };
	m_vecBridgeCallbacks["VERSION"] = { std::bind( &MessageProcessor::OnVersion, this, std::placeholders::_1 ), 0 };
	m_vecBridgeCallbacks["PAUSE"] = { std::bind( &MessageProcessor::OnPause, this, std::placeholders::_1 ), 1 };
	m_vecBridgeCallbacks["REENABLE"] = { std::bind( &MessageProcessor::OnReEnable, this, std::placeholders::_1 ), 0 };
}

MessageProcessor::~MessageProcessor( )
{

}

//////////////////////////////////////////////////////////////////////////
// Brain Stem Callbacks
//////////////////////////////////////////////////////////////////////////

void MessageProcessor::ProcessMessage( std::vector<char> buffer )
{
	BRAIN_STEM_MSG eCommand = BRAIN_STEM_MSG::UNKNOWN;

	const char* pszData = buffer.data( );

	int size = buffer.size( );

	if( buffer.size( ) > 0 )
	{
		eCommand = static_cast<BRAIN_STEM_MSG>( buffer[0] );
	}

	switch( eCommand )
	{
		case BRAIN_STEM_MSG::MESSAGE:
		{
			std::string strMessage( buffer.begin( ), buffer.end( ) );

			if( strMessage.find( "<MSG Error" ) != -1 )
			{
				ROS_ERROR_NAMED( "Brainstem", "Fatal Error: %s", strMessage.c_str( ) );

				m_bControllerFault = true;
			}
			else
			{
				ROS_DEBUG_NAMED( "Brainstem", "%s", strMessage.c_str( ) );
			}
		}
		break;

		case BRAIN_STEM_MSG::STOP:
		{
			std_msgs::String msg;
			msg.data = "ARRIVED";

			ROS_DEBUG_NAMED( "Brainstem", "%s", msg.data.c_str( ) );

			m_llEventPublisher.publish( msg );
		}
		break;

		case BRAIN_STEM_MSG::BUTTON:
		{
			ENTITIES eButtonId = static_cast<ENTITIES>( buffer[1] );

			std::string strEntity;

			auto iter = m_mapEntityButton.find( eButtonId );

			if( iter != m_mapEntityButton.end( ) )
			{
				std_msgs::String msg;

				std::stringstream ss;
				ss << "UI " << iter->second;
				msg.data = ss.str();

				ROS_DEBUG_NAMED( "Brainstem", "%s", msg.data.c_str( ) );

				m_llEventPublisher.publish( msg );
			}
			else
			{
				ROS_ERROR_NAMED( "Brainstem", "Unknown button entity %d", eButtonId );
			}
		}
		break;

		case BRAIN_STEM_MSG::ODOMETRY:
		{
			ODOMETRY_DATA* pOdometry = reinterpret_cast<ODOMETRY_DATA*>( buffer.data( ) );

			ros::Time currentTime = ros::Time::now( );

			static ros::Time sLastTime = currentTime;

			bool bInvalidTime = ( currentTime.toSec( ) - m_rosOdomTime.toSec( ) ) > 0.1;

			if( !m_rosOdomTime.isZero( ) &&
				bInvalidTime )
			{
				ROS_ERROR_NAMED( "Brainstem", "Timestamp out of range and resynced (possible communication problem): odom: %f, ros: %f",
					m_rosOdomTime.toSec( ), currentTime.toSec( ) );
			}

			if( bInvalidTime ||
				( approximatively_equal( pOdometry->linear_velocity, 0.0f, 0.00001 ) &&
				  approximatively_equal( pOdometry->angular_velocity, 0.0f, 0.00001 ) ) )
			{
				// Reset our time basis to account for any drift
				m_rosOdomTime = currentTime - ros::Duration( 0, 1200000 );
			}
			else
			{
				double dfOdomTimeDelta = (double)(pOdometry->timestamp - m_dwLastOdomTime) / 1000.0f;

				ROS_DEBUG_NAMED( "Brainstem", "Odometry (%f): %f, %f",
					dfOdomTimeDelta, pOdometry->linear_velocity, pOdometry->angular_velocity );

				// Base our time on the realtime clock (brain_stem) since our clock does not match odom info
				// and our loop is not realtime
				m_rosOdomTime += ros::Duration( dfOdomTimeDelta );
			}

			geometry_msgs::TwistStamped odometry;
			odometry.header.stamp = m_rosOdomTime;
			odometry.twist.linear.x = pOdometry->linear_velocity;
			odometry.twist.angular.z = pOdometry->angular_velocity;

			m_OdometryRawPublisher.publish( odometry );

			double dfClockDiff = currentTime.toSec( ) - m_rosOdomTime.toSec( );

			if( dfClockDiff > 0.05 )
			{
				ROS_ERROR_NAMED( "Brainstem", "Odometry clock drift: %f", dfClockDiff );
			}

			m_dwLastOdomTime = pOdometry->timestamp;

			sLastTime = currentTime;
		}
		break;

		case BRAIN_STEM_MSG::UNKNOWN:
		default:
		{
			ROS_ERROR_STREAM_NAMED( "Brainstem", "Unknown message from brainstem: " << (int)eCommand << ", data: " << ToHex( buffer ) );
		}
		break;
	}
}

void MessageProcessor::OnChangeVelocity( const geometry_msgs::Twist::ConstPtr& velocity )
{
	VELOCITY_DATA msg = {
		static_cast<uint8_t>( BRAIN_STEM_CMD::SET_VELOCITY ),
		static_cast<float>( velocity->linear.x ),
		static_cast<float>( velocity->angular.z )
	};

	static geometry_msgs::Twist s_currentVelocity;

	if( velocity->linear.x != s_currentVelocity.linear.x ||
		velocity->angular.z != s_currentVelocity.angular.z )
	{
		ROS_DEBUG_NAMED( "Brainstem", "Velocity: %f, %f",
			velocity->linear.x, velocity->angular.z );

		s_currentVelocity = *velocity;
	}

	// Send the velocity down to the motors
	WriteToSerialPort( reinterpret_cast<char*>( &msg ), sizeof( msg ) );
}

//////////////////////////////////////////////////////////////////////////
// ROS Callbacks
//////////////////////////////////////////////////////////////////////////

void MessageProcessor::OnRosCallback( const std_msgs::String::ConstPtr& msg )
{
	const std::string& strMessage = msg->data;

	ROS_DEBUG_NAMED( "Brainstem", "Received command: %s", strMessage.c_str( ) );

	std::vector<std::string> vecParsed;
	boost::tokenizer<> tok( strMessage );

	for( auto && strValue : tok ) { vecParsed.push_back( strValue ); }

	if( vecParsed.size( ) )
	{
		const std::string& strCommand = vecParsed[0];

		std::vector<std::string> vecParams = { vecParsed.begin( ) + 1, vecParsed.end( ) };

		auto iter = m_vecBridgeCallbacks.find( strCommand );

		if( iter != m_vecBridgeCallbacks.end( ) )
		{
			if( vecParams.size( ) == iter->second.dwNumParams )
			{
				iter->second.callback( vecParams );
			}
			else
			{
				ROS_ERROR_NAMED( "Brainstem", "Bridge has invalid number of arguments: %s", strMessage.c_str( ) );
			}
		}
	}
}


void MessageProcessor::SetConnected( bool bIsConnected )
{
	std_msgs::Bool msg;
	msg.data = bIsConnected;

	m_ConnectedPublisher.publish( msg );

	m_rosOdomTime = ros::Time( );
}

//////////////////////////////////////////////////////////////////////////
// Bridge Callbacks
//////////////////////////////////////////////////////////////////////////

void MessageProcessor::OnUI( std::vector<std::string> vecParams )
{
	std::string strEntity = vecParams[0];

	std::string strMode = vecParams[1];

	auto iterEntity = m_mapButtonEntity.find( strEntity );

	LIGHT_UPDATE_DATA msg = {
		static_cast<uint8_t>( BRAIN_STEM_CMD::LIGHT_UPDATE ),
		static_cast<uint8_t>( ENTITIES::UNKNOWN ),
		static_cast<uint8_t>( LED_MODE::UNKNOWN )
	};

	if( iterEntity != m_mapButtonEntity.end( ) )
	{
		msg.entitiy = static_cast<uint8_t>( iterEntity->second );
	}
	else
	{
		ROS_ERROR( "Invalid button entity name: %s", strEntity.c_str( ) );
	}

	auto iterMode = m_mapLedMode.find( strMode );

	if( iterMode != m_mapLedMode.end( ) )
	{
		msg.mode = static_cast<uint8_t>( iterMode->second );
	}
	else
	{
		ROS_ERROR( "Invalid button entity name: %s", strEntity.c_str( ) );
	}

	if( static_cast<ENTITIES>( msg.entitiy ) != ENTITIES::UNKNOWN &&
		static_cast<LED_MODE>( msg.mode ) != LED_MODE::UNKNOWN )
	{
		WriteToSerialPort( reinterpret_cast<char*>( &msg ), sizeof( msg ) );
	}
}

void MessageProcessor::OnStartup( std::vector<std::string> vecParams )
{
	uint8_t cMessage = static_cast<uint8_t>( BRAIN_STEM_CMD::STARTUP );

	WriteToSerialPort( reinterpret_cast<char*>( &cMessage ), 1 );
}

void MessageProcessor::OnVersion( std::vector<std::string> vecParams )
{
	uint8_t cMessage = static_cast<uint8_t>( BRAIN_STEM_CMD::GET_VERSION );

	WriteToSerialPort( reinterpret_cast<char*>( &cMessage ), 1 );
}

void MessageProcessor::OnPause( std::vector<std::string> vecParams )
{
	std::string& strPaused = vecParams[0];

	SUSPEND_DATA msg = { static_cast<uint8_t>( BRAIN_STEM_CMD::SUSPEND_UPDATE_STATE ),
		strPaused == "OFF" ? false : true };

	WriteToSerialPort( reinterpret_cast<char*>( &msg ), sizeof( msg ) );
}

void MessageProcessor::OnReEnable( std::vector<std::string> vecParams )
{
	ROS_INFO_NAMED( "BrainStem", "ReEnable from fault mode" );

	m_bControllerFault = false;
}

//////////////////////////////////////////////////////////////////////////
// Helper Methods
//////////////////////////////////////////////////////////////////////////

void MessageProcessor::WriteToSerialPort( char* pszData, std::size_t dwSize )
{
	if( !m_bControllerFault )
	{
		if( m_pIO->IsOpen( ) )
		{
			m_pIO->Write( std::vector<char>( pszData, pszData + dwSize ) );
		}
		else
		{
			ROS_ERROR_THROTTLE_NAMED( 60, "BrainStem", "Attempt to write to the brain stem, but the serial port is not open!" );
		}
	}
}

} /* namespace srs */
