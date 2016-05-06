/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#include <BrainStemMessageProcessor.h>
#include <BrainStemMessages.h>
#include <srslib_framework/io/IO.hpp>
#include <srslib_framework/utils/Logging.hpp>

#include <chrono>
#include <boost/tokenizer.hpp>
#include <ros/ros.h>

namespace srs {

using namespace ros;

BrainStemMessageProcessor::BrainStemMessageProcessor( std::shared_ptr<IO> pIO ) :
	m_bControllerFault( false ),
	m_pIO( pIO )
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

	m_vecBridgeCallbacks["UI"] = { std::bind( &BrainStemMessageProcessor::OnUI, this, std::placeholders::_1 ), 2 };
	m_vecBridgeCallbacks["STARTUP"] = { std::bind( &BrainStemMessageProcessor::OnStartup, this, std::placeholders::_1 ), 0 };
	m_vecBridgeCallbacks["VERSION"] = { std::bind( &BrainStemMessageProcessor::OnVersion, this, std::placeholders::_1 ), 0 };
	m_vecBridgeCallbacks["PAUSE"] = { std::bind( &BrainStemMessageProcessor::OnPause, this, std::placeholders::_1 ), 1 };
	m_vecBridgeCallbacks["REENABLE"] = { std::bind( &BrainStemMessageProcessor::OnReEnable, this, std::placeholders::_1 ), 0 };
}

BrainStemMessageProcessor::~BrainStemMessageProcessor( )
{

}

//////////////////////////////////////////////////////////////////////////
// Message Callbacks
//////////////////////////////////////////////////////////////////////////

void BrainStemMessageProcessor::SetConnectionChangedCallback( ConnectionChangedFn connectionChangedCallback )
{
	m_connectionChangedCallback = connectionChangedCallback;
}

void BrainStemMessageProcessor::SetArrivedCallback( ArrivedCallbackFn arrivedCallback )
{
	m_arrivedCallback = arrivedCallback;
}

void BrainStemMessageProcessor::SetButtonCallback( ButtonCallbackFn buttonCallback )
{
	m_buttonCallback = buttonCallback;
}

void BrainStemMessageProcessor::SetOdometryCallback( OdometryCallbackFn odometryCallback )
{
	m_odometryCallback = odometryCallback;
}

//////////////////////////////////////////////////////////////////////////
// Message Processing
//////////////////////////////////////////////////////////////////////////

void BrainStemMessageProcessor::ProcessBrainStemMessage( std::vector<char> buffer )
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
			m_arrivedCallback( );
		}
		break;

		case BRAIN_STEM_MSG::BUTTON:
		{
			ENTITIES eButtonId = static_cast<ENTITIES>( buffer[1] );

			m_buttonCallback( eButtonId );
		}
		break;

		case BRAIN_STEM_MSG::ODOMETRY:
		{
			ODOMETRY_DATA* pOdometry = reinterpret_cast<ODOMETRY_DATA*>( buffer.data( ) );

			m_odometryCallback( pOdometry->timestamp, pOdometry->linear_velocity, pOdometry->angular_velocity );
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

void BrainStemMessageProcessor::SetVelocity( double dfLinear, double dfAngular )
{
	VELOCITY_DATA msg = {
		static_cast<uint8_t>( BRAIN_STEM_CMD::SET_VELOCITY ),
		static_cast<float>( dfLinear ),
		static_cast<float>( dfAngular )
	};

	static double s_dfLinear = dfLinear;
	static double s_dfAngular = dfAngular;

	if( dfLinear != s_dfLinear ||
		dfAngular != s_dfAngular )
	{
		ROS_DEBUG_NAMED( "Brainstem", "Velocity: %f, %f", dfLinear, dfAngular );

		s_dfLinear = dfLinear;
		s_dfAngular = dfAngular;
	}

	// Send the velocity down to the motors
	WriteToSerialPort( reinterpret_cast<char*>( &msg ), sizeof( msg ) );
}

////////////////////////////////////////////////////////////////////////////
//// ROS Callbacks
////////////////////////////////////////////////////////////////////////////

void BrainStemMessageProcessor::ProcessRosMessage( const std::string& strMessage )
{
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

void BrainStemMessageProcessor::SetConnected( bool bIsConnected )
{
	m_connectionChangedCallback( bIsConnected );
}

//////////////////////////////////////////////////////////////////////////
// Helper
//////////////////////////////////////////////////////////////////////////

std::string BrainStemMessageProcessor::GetButtonName( ENTITIES eButtonId ) const
{
	std::string strName;

	auto iter = m_mapEntityButton.find( eButtonId );

	if( iter != m_mapEntityButton.end( ) )
	{
		strName = iter->second;
	}

	return strName;
}

//////////////////////////////////////////////////////////////////////////
// Bridge Callbacks
//////////////////////////////////////////////////////////////////////////

void BrainStemMessageProcessor::OnUI( std::vector<std::string> vecParams )
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

void BrainStemMessageProcessor::OnStartup( std::vector<std::string> vecParams )
{
	uint8_t cMessage = static_cast<uint8_t>( BRAIN_STEM_CMD::STARTUP );

	WriteToSerialPort( reinterpret_cast<char*>( &cMessage ), 1 );
}

void BrainStemMessageProcessor::OnVersion( std::vector<std::string> vecParams )
{
	uint8_t cMessage = static_cast<uint8_t>( BRAIN_STEM_CMD::GET_VERSION );

	WriteToSerialPort( reinterpret_cast<char*>( &cMessage ), 1 );
}

void BrainStemMessageProcessor::OnPause( std::vector<std::string> vecParams )
{
	std::string& strPaused = vecParams[0];

	SUSPEND_DATA msg = { static_cast<uint8_t>( BRAIN_STEM_CMD::SUSPEND_UPDATE_STATE ),
		strPaused == "OFF" ? false : true };

	WriteToSerialPort( reinterpret_cast<char*>( &msg ), sizeof( msg ) );
}

void BrainStemMessageProcessor::OnReEnable( std::vector<std::string> vecParams )
{
	ROS_INFO_NAMED( "BrainStem", "ReEnable from fault mode" );

	m_bControllerFault = false;
}

//////////////////////////////////////////////////////////////////////////
// Helper Methods
//////////////////////////////////////////////////////////////////////////

void BrainStemMessageProcessor::WriteToSerialPort( char* pszData, std::size_t dwSize )
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
