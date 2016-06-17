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
	m_pIO( pIO )
{
	m_mapEntityButton[LED_ENTITIES::TOTE0]		= "TOTE0";
	m_mapEntityButton[LED_ENTITIES::TOTE1]		= "TOTE1";
	m_mapEntityButton[LED_ENTITIES::TOTE2]		= "TOTE2";
	m_mapEntityButton[LED_ENTITIES::TOTE3]		= "TOTE3";
	m_mapEntityButton[LED_ENTITIES::TOTE4]		= "TOTE4";
	m_mapEntityButton[LED_ENTITIES::TOTE5]		= "TOTE5";
	m_mapEntityButton[LED_ENTITIES::TOTE6]		= "TOTE6";
	m_mapEntityButton[LED_ENTITIES::TOTE7]		= "TOTE7";
	m_mapEntityButton[LED_ENTITIES::ACTION]		= "ACTION";
	m_mapEntityButton[LED_ENTITIES::PAUSE]		= "PAUSE";
	m_mapEntityButton[LED_ENTITIES::TAIL_LEFT]	= "TAIL_LEFT";
	m_mapEntityButton[LED_ENTITIES::TAIL_RIGHT]	= "TAIL_RIGHT";

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

	m_vecBridgeCallbacks["UI"] = { std::bind( &BrainStemMessageProcessor::OnUpdateLights, this, std::placeholders::_1 ), 2 };
	m_vecBridgeCallbacks["STOP"] = { std::bind( &BrainStemMessageProcessor::OnHardStop, this ), 0 };
	m_vecBridgeCallbacks["STARTUP"] = { std::bind( &BrainStemMessageProcessor::OnStartup, this, std::placeholders::_1 ), 0 };
	m_vecBridgeCallbacks["PAUSE"] = { std::bind( &BrainStemMessageProcessor::OnPause, this, std::placeholders::_1 ), 1 };
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
		case BRAIN_STEM_MSG::BUTTON_PRESSED:
		{
			LED_ENTITIES eButtonId = static_cast<LED_ENTITIES>( buffer[1] );

			if( m_mapEntityButton.find( eButtonId ) != m_mapEntityButton.end( ) )
			{
				m_buttonCallback( eButtonId );
			}
			else
			{
				ROS_ERROR_STREAM( "Unknown button pressed: " << (int)eButtonId );
			}
		}
		break;

		case BRAIN_STEM_MSG::HARDWARE_INFO:
		{
			HARDWARE_INFORMATION_DATA* pHardwareInfo = reinterpret_cast<HARDWARE_INFORMATION_DATA*>( buffer.data( ) );

			std::string strBrainstemVersion( pHardwareInfo->pszBrainstemVersion );

			std::string strSafetyProcessorVersion( pHardwareInfo->pszBrainstemVersion + strBrainstemVersion.size( ) + 1 );

			ROS_INFO_STREAM( "Hardware Info => id:" << pHardwareInfo->uniqueId << ", bodyType: " << pHardwareInfo->bodyType <<
				", configuration:" << pHardwareInfo->configuration << ", lifetimeHours:" << pHardwareInfo->lifetimeHours << ", lifetimeMeters:" << pHardwareInfo->lifetimeMeters <<
				", batteryHours:" << pHardwareInfo->batteryHours << ", wheelMeters:" << pHardwareInfo->wheelMeters << ", Brainstem Version:" << strBrainstemVersion <<
				", Safety Processor Version:" << strSafetyProcessorVersion );
		}
		break;

		case BRAIN_STEM_MSG::OPERATIONAL_STATE:
		{
			OPERATIONAL_STATE_DATA* pOperationalState = reinterpret_cast<OPERATIONAL_STATE_DATA*>( buffer.data( ) );

			ROS_INFO_STREAM( "Hardware Info => id:" << pOperationalState->upTime <<
				", frontEStop: " << pOperationalState->motionStatus.frontEStop << ", backEStop: " << pOperationalState->motionStatus.backEStop <<
				", wirelessEStop: " << pOperationalState->motionStatus.wirelessEStop << ", bumpSensor: " << pOperationalState->motionStatus.bumpSensor <<
				", pause: " << pOperationalState->motionStatus.pause << ", hardStop: " << pOperationalState->motionStatus.hardStop <<
				", safetyProcessorFailure: " << pOperationalState->failureStatus.safetyProcessorFailure << ", brainstemFailure: " << pOperationalState->failureStatus.brainstemFailure <<
				", brainTimeoutFailure: " << pOperationalState->failureStatus.brainTimeoutFailure << ", rightMotorFailure: " << pOperationalState->failureStatus.rightMotorFailure <<
				", leftMotorFailure: " << pOperationalState->failureStatus.leftMotorFailure << ", suspendState: " << pOperationalState->suspendState );
		}
		break;

		case BRAIN_STEM_MSG::SYSTEM_VOLTAGE:
		{
			VOLTAGE_DATA* pVoltage = reinterpret_cast<VOLTAGE_DATA*>( buffer.data( ) );

			ROS_INFO_STREAM( "Voltage => " << pVoltage->voltage);
		}
		break;

		case BRAIN_STEM_MSG::ODOMETRY_VELOCITY:
		{
			ODOMETRY_DATA* pOdometry = reinterpret_cast<ODOMETRY_DATA*>( buffer.data( ) );

			m_odometryCallback( pOdometry->timestamp, pOdometry->linear_velocity, pOdometry->angular_velocity );
		}
		break;

		case BRAIN_STEM_MSG::UNKNOWN:
		default:
		{
			ROS_ERROR_STREAM( "Unknown message from brainstem: " << (int)eCommand << ", data: " << ToHex( buffer ) );
		}
		break;
	}
}

void BrainStemMessageProcessor::GetOperationalState( )
{
	COMMAND_DATA msg = { static_cast<uint8_t>( BRAIN_STEM_CMD::GET_OPERATIONAL_STATE) };

	// Get the operational state
	WriteToSerialPort( reinterpret_cast<char*>( &msg ), sizeof( msg ) );
}

void BrainStemMessageProcessor::GetHardwareInformation( )
{
	COMMAND_DATA msg = { static_cast<uint8_t>( BRAIN_STEM_CMD::GET_HARDWARE_INFO ) };

	// Get the hardware information (version, configuration, etc.)
	WriteToSerialPort( reinterpret_cast<char*>( &msg ), sizeof( msg ) );
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
		ROS_DEBUG( "Velocity: %f, %f", dfLinear, dfAngular );

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
	boost::char_separator<char> separator(";");

	boost::tokenizer<boost::char_separator<char>> messageListTokenizer( strMessage, separator );

	// Parse the list of semicolon delimited commands
	for( auto && strCommand : messageListTokenizer )
	{
		ROS_DEBUG( "Parse command: %s", strCommand.c_str( ) );

		// Parse the command by whitespace
		std::vector<std::string> vecParsed;
		boost::tokenizer<> tok( strCommand );

		for( auto && strValue : tok )
		{
			vecParsed.push_back( strValue );
		}

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
}

void BrainStemMessageProcessor::SetConnected( bool bIsConnected )
{
	m_connectionChangedCallback( bIsConnected );
}

void BrainStemMessageProcessor::Shutdown( )
{
	uint8_t cMessage = static_cast<uint8_t>( BRAIN_STEM_CMD::SHUTDOWN );

	WriteToSerialPort( reinterpret_cast<char*>( &cMessage ), 1 );
}

//////////////////////////////////////////////////////////////////////////
// Helper
//////////////////////////////////////////////////////////////////////////

std::string BrainStemMessageProcessor::GetButtonName( LED_ENTITIES eButtonId ) const
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

void BrainStemMessageProcessor::OnClearMotionStatus( MOTION_STATUS eMotionStatus )
{
	MOTION_STATUS_DATA statusData = { false };
	statusData.frontEStop = (eMotionStatus == MOTION_STATUS::FRONT_E_STOP);
	statusData.backEStop = (eMotionStatus == MOTION_STATUS::BACK_E_STOP);
	statusData.wirelessEStop = (eMotionStatus == MOTION_STATUS::WIRELESS_E_STOP);
	statusData.bumpSensor = (eMotionStatus == MOTION_STATUS::BUMP_SENSOR);
	statusData.pause = (eMotionStatus == MOTION_STATUS::PAUSE);
	statusData.hardStop = (eMotionStatus == MOTION_STATUS::HARD_STOP);

	SET_OPERATIONAL_STATE_DATA msg = { static_cast<uint8_t>( BRAIN_STEM_CMD::SET_MOTION_STATUS ), statusData };

	WriteToSerialPort( reinterpret_cast<char*>( &msg ), sizeof( msg ) );
}

void BrainStemMessageProcessor::OnHardStop( )
{
	uint8_t cMessage = static_cast<uint8_t>( BRAIN_STEM_CMD::HARD_STOP );

	WriteToSerialPort( reinterpret_cast<char*>( &cMessage ), 1 );
}

void BrainStemMessageProcessor::OnPing( )
{
	uint8_t cMessage = static_cast<uint8_t>( BRAIN_STEM_CMD::PING );

	WriteToSerialPort( reinterpret_cast<char*>( &cMessage ), 1 );
}

void BrainStemMessageProcessor::OnResetBatteryHours( )
{
	uint8_t cMessage = static_cast<uint8_t>( BRAIN_STEM_CMD::RESET_BATTERY_HOURS );

	WriteToSerialPort( reinterpret_cast<char*>( &cMessage ), 1 );
}

void BrainStemMessageProcessor::OnResetWheelMeters( )
{
	uint8_t cMessage = static_cast<uint8_t>( BRAIN_STEM_CMD::RESET_WHEEL_METERS );

	WriteToSerialPort( reinterpret_cast<char*>( &cMessage ), 1 );
}

void BrainStemMessageProcessor::OnSetConfiguration( uint32_t configuration )
{
	uint8_t cMessage = static_cast<uint8_t>( BRAIN_STEM_CMD::SET_CONFIGURATION );

	WriteToSerialPort( reinterpret_cast<char*>( &cMessage ), 1 );
}

void BrainStemMessageProcessor::OnUpdateLights( std::vector<std::string> vecParams )
{
	std::string strEntity = vecParams[0];

	std::string strMode = vecParams[1];

	auto iterEntity = m_mapButtonEntity.find( strEntity );

	LIGHT_UPDATE_DATA msg = {
		static_cast<uint8_t>( BRAIN_STEM_CMD::UPDATE_LIGHT ),
		static_cast<uint8_t>( LED_ENTITIES::UNKNOWN ),
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

	if( static_cast<LED_ENTITIES>( msg.entitiy ) != LED_ENTITIES::UNKNOWN &&
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

void BrainStemMessageProcessor::OnPause( std::vector<std::string> vecParams )
{
	std::string& strPaused = vecParams[0];

	SUSPEND_DATA msg = { static_cast<uint8_t>( BRAIN_STEM_CMD::SET_SUSPEND_STATE ),
		strPaused == "OFF" ? false : true };

	WriteToSerialPort( reinterpret_cast<char*>( &msg ), sizeof( msg ) );
}

//////////////////////////////////////////////////////////////////////////
// Helper Methods
//////////////////////////////////////////////////////////////////////////

void BrainStemMessageProcessor::WriteToSerialPort( char* pszData, std::size_t dwSize )
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

} /* namespace srs */
