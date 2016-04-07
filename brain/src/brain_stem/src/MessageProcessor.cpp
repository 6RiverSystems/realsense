/*
 * MessageProcessor.cpp
 *
 *  Created on: Apr 7, 2016
 *      Author: dan
 */

#include "MessageProcessor.h"
#include "Messages.h"
#include "IO.h"

#include <chrono>
#include <boost/tokenizer.hpp>

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>

namespace srs {

using namespace ros;

MessageProcessor::MessageProcessor( ros::NodeHandle& node, IO* pIO ) :
	m_bControllerFault( false ),
	m_node( node ),
	m_pIO( pIO ),
	m_llcmdSubscriber( node.subscribe<std_msgs::String>( "/cmd_ll", 1000,
		std::bind( &MessageProcessor::OnRosCallback, this, std::placeholders::_1 ) ) ),
	m_llEventPublisher( node.advertise<std_msgs::String>( "/ll_event", 50 ) ),
	m_llSensorsPublisher( node.advertise<std_msgs::String>( "/ll_sensors", 1000 ) )
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
	m_vecBridgeCallbacks["DISTANCE"] = { std::bind( &MessageProcessor::OnDistance, this, std::placeholders::_1 ), 1 };
	m_vecBridgeCallbacks["ROTATE"] = { std::bind( &MessageProcessor::OnRotate, this, std::placeholders::_1 ), 1 };
	m_vecBridgeCallbacks["STOP"] = { std::bind( &MessageProcessor::OnStop, this, std::placeholders::_1 ), 0 };
	m_vecBridgeCallbacks["TURN"] = { std::bind( &MessageProcessor::OnTurn, this, std::placeholders::_1 ), 2 };
	m_vecBridgeCallbacks["VERSION"] = { std::bind( &MessageProcessor::OnVersion, this, std::placeholders::_1 ), 0 };
	m_vecBridgeCallbacks["PAUSE"] = { std::bind( &MessageProcessor::OnPause, this, std::placeholders::_1 ), 1 };
	m_vecBridgeCallbacks["REENABLE"] = { std::bind( &MessageProcessor::OnReEnable, this, std::placeholders::_1 ), 0 };
	m_vecBridgeCallbacks["SET_PID"] = { std::bind( &MessageProcessor::OnSetPid, this, std::placeholders::_1 ), 4 };
}

MessageProcessor::~MessageProcessor( )
{

}

//////////////////////////////////////////////////////////////////////////
// Brain Stem Callbacks
//////////////////////////////////////////////////////////////////////////

void MessageProcessor::ProcessMessage( std::vector<char> buffer )
{
	BRAIN_STEM_MSG eCommand;

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
				ROS_ERROR( "Fatal Error: %s", strMessage.c_str( ) );

				m_bControllerFault = true;

				// TODO: Send event
			}
			else
			{
				ROS_DEBUG( "%s", strMessage.c_str( ) );
			}
		}
		break;

		case BRAIN_STEM_MSG::STOP:
		{
			std_msgs::String message;
			message.data = "ARRIVED";

			m_llEventPublisher.publish( message );
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

				m_llEventPublisher.publish( msg );
			}
			else
			{
				ROS_ERROR( "Unknown button entity %d", eButtonId );
			}
		}
		break;

		case BRAIN_STEM_MSG::ODOMETRY:
		{
			ODOMETRY_DATA* pOdometry = reinterpret_cast<ODOMETRY_DATA*>( buffer.data( ) + 1 );

			std_msgs::String msg;

            double fractional_seconds_since_epoch = std::chrono::duration_cast<std::chrono::duration<double>>(
            	std::chrono::system_clock::now( ).time_since_epoch( ) ).count( );

			std::stringstream ss;
			ss << "O " << pOdometry->left_encoder << "," << pOdometry->right_encoder << " @ " << fractional_seconds_since_epoch;
			msg.data = ss.str();

			m_llEventPublisher.publish( msg );
		}
		break;

		case BRAIN_STEM_MSG::PID:
		{
			// TODO: Define real pid messages to help with tuning
			PID_DATA* pPid = reinterpret_cast<PID_DATA*>( buffer.data( ) + 1 );
			// pubXEst.publish("X_DESIRED %f" % pidData[1])
			// pubXEst.publish("X_CURRENT %f" % pidData[1])
		}
		break;

		default:
		{
			ROS_DEBUG( "Unknown MFP command: %d", eCommand );
		}
		break;
	}
}

//////////////////////////////////////////////////////////////////////////
// ROS Callbacks
//////////////////////////////////////////////////////////////////////////

void MessageProcessor::OnRosCallback( const std_msgs::String::ConstPtr& msg )
{
	const std::string& strMessage = msg->data;

	ROS_DEBUG( "Received command: %s", strMessage.c_str( ) );

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
				ROS_ERROR( "Bridge has invalid number of arguments: %s", strMessage.c_str( ) );
			}
		}
	}
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

	WriteToSerialPort( reinterpret_cast<char*>( &cMessage ), sizeof( cMessage ) );
}

void MessageProcessor::OnDistance( std::vector<std::string> vecParams )
{
	MOVE_DISTANCE_DATA msg = {
		static_cast<uint8_t>( BRAIN_STEM_CMD::DISTANCE ),
		static_cast<uint8_t>( boost::lexical_cast<int>( vecParams[0] ) )
	};

	WriteToSerialPort( reinterpret_cast<char*>( &msg ), sizeof( msg ) );
}

void MessageProcessor::OnRotate( std::vector<std::string> vecParams )
{
	ROTATE_DATA msg = {
		static_cast<uint8_t>( BRAIN_STEM_CMD::ROTATE ),
		static_cast<uint8_t>( boost::lexical_cast<int>( vecParams[0] ) )
	};

	WriteToSerialPort( reinterpret_cast<char*>( &msg ), sizeof( msg ) );
}

void MessageProcessor::OnStop( std::vector<std::string> vecParams )
{
	uint8_t cMessage = static_cast<uint8_t>( BRAIN_STEM_CMD::STARTUP );

	WriteToSerialPort( reinterpret_cast<char*>( &cMessage ), sizeof( cMessage ) );
}

void MessageProcessor::OnTurn( std::vector<std::string> vecParams )
{
	std::string& strDirection = vecParams[0];

	std::string& strDistance = vecParams[1];

	BRAIN_STEM_CMD cCommand = BRAIN_STEM_CMD::UNKNOWN;

	if( strDirection == "R" )
	{
		cCommand = BRAIN_STEM_CMD::SLINGSHOT_RIGHT;
	}
	else if( strDirection == "L" )
	{
		cCommand = BRAIN_STEM_CMD::SLINGSHOT_LEFT;
	}

	SLINGSHOT_DATA msg = { static_cast<uint8_t>( cCommand ), boost::lexical_cast<uint16_t>( strDistance ) };

	WriteToSerialPort( reinterpret_cast<char*>( &msg ), sizeof( msg ) );
}

void MessageProcessor::OnVersion( std::vector<std::string> vecParams )
{
	uint8_t cMessage = static_cast<uint8_t>( BRAIN_STEM_CMD::GET_VERSION );

	WriteToSerialPort( reinterpret_cast<char*>( &cMessage ), sizeof( cMessage ) );
}

void MessageProcessor::OnPause( std::vector<std::string> vecParams )
{
	std::string& strPaused = vecParams[0];

	SUSPEND_DATA msg = { static_cast<uint8_t>( BRAIN_STEM_CMD::SUSPEND_UPDATE_STATE ),
		strPaused == "OFF" ? true : false };

	WriteToSerialPort( reinterpret_cast<char*>( &msg ), sizeof( msg ) );
}

void MessageProcessor::OnReEnable( std::vector<std::string> vecParams )
{
	m_bControllerFault = false;
}

void MessageProcessor::OnSetPid( std::vector<std::string> vecParams )
{
	float fKp = boost::lexical_cast<int>( vecParams[0] );
	float fKi = boost::lexical_cast<int>( vecParams[1] );
	float fKd = boost::lexical_cast<int>( vecParams[2] );
	float proj_time = boost::lexical_cast<int>( vecParams[3] );

	PID_DATA msg = { static_cast<uint8_t>( BRAIN_STEM_CMD::SET_PID ),
		fKp, fKi, fKd, proj_time };

	WriteToSerialPort( reinterpret_cast<char*>( &msg ), sizeof( msg ) );
}

//////////////////////////////////////////////////////////////////////////
// Helper Methods
//////////////////////////////////////////////////////////////////////////

void MessageProcessor::WriteToSerialPort( char* pszData, std::size_t dwSize )
{
	if( !m_bControllerFault )
	{
		m_pIO->Write( std::vector<char>( pszData, pszData + dwSize ) );
	}
}

} /* namespace srs */
