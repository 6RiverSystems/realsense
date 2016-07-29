/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#include <srsdrv_brainstem/BrainStemMessageProcessor.h>

#include <chrono>
#include <boost/tokenizer.hpp>

#include <ros/ros.h>

#include <srslib_framework/io/IO.hpp>
#include <srslib_framework/utils/Logging.hpp>

#include <srsdrv_brainstem/BrainStemMessages.h>

namespace srs {

using namespace ros;

BrainStemMessageProcessor::BrainStemMessageProcessor( std::shared_ptr<IO> pIO ) :
	m_pIO( pIO ),
	m_mapMotionStatus( )
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
	m_mapEntityButton[LED_ENTITIES::SCANNER]	= "SCANNER";

	m_mapLedMode["OFF"] 					= LED_MODE::OFF;
	m_mapLedMode["ON"] 						= LED_MODE::ON;
	m_mapLedMode["ESTOP"] 					= LED_MODE::ESTOP;
	m_mapLedMode["GRAB"] 					= LED_MODE::GRAB;
	m_mapLedMode["PUT"] 					= LED_MODE::PUT;
	m_mapLedMode["BRAKE"] 					= LED_MODE::BRAKE;
	m_mapLedMode["TURN"] 					= LED_MODE::TURN;
	m_mapLedMode["SELECT"] 					= LED_MODE::SELECT;
	m_mapLedMode["RAPID_BLINK"] 			= LED_MODE::RAPID_BLINK;

	m_mapMotionStatus["WIRELESS_E_STOP"]	= MOTION_STATUS::WIRELESS_E_STOP;
	m_mapMotionStatus["BUMP_SENSOR"]		= MOTION_STATUS::BUMP_SENSOR;
	m_mapMotionStatus["PAUSE"]				= MOTION_STATUS::PAUSE;
	m_mapMotionStatus["HARD_STOP"]			= MOTION_STATUS::HARD_STOP;

	for( auto& kv : m_mapEntityButton )
	{
		m_mapButtonEntity[kv.second] = kv.first;
	}

	m_vecBridgeCallbacks["UI"] = { std::bind( &BrainStemMessageProcessor::OnUpdateLights, this, std::placeholders::_1 ), 2 };
	m_vecBridgeCallbacks["STOP"] = { std::bind( &BrainStemMessageProcessor::OnHardStop, this ), 0 };
	m_vecBridgeCallbacks["STARTUP"] = { std::bind( &BrainStemMessageProcessor::OnStartup, this, std::placeholders::_1 ), 0 };
	m_vecBridgeCallbacks["PAUSE"] = { std::bind( &BrainStemMessageProcessor::OnPause, this, std::placeholders::_1 ), 1 };
	m_vecBridgeCallbacks["CLEAR_MOTION_STATUS"] = { std::bind( &BrainStemMessageProcessor::ClearMotionStatus, this ), 0 };

    hwMessageHandlers_.push_back(&sensorFrameHandler_);
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

void BrainStemMessageProcessor::SetButtonCallback( ButtonCallbackFn buttonCallback )
{
	m_buttonCallback = buttonCallback;
}

void BrainStemMessageProcessor::SetHardwareInfoCallback( HardwareInfoCallbackFn hardwareInfoCallback )
{
	m_hardwareInfoCallback = hardwareInfoCallback;
}

void BrainStemMessageProcessor::SetOperationalStateCallback( OperationalStateCallbackFn operationalStateCallback )
{
	m_operationalStateCallback = operationalStateCallback;
}

void BrainStemMessageProcessor::SetVoltageCallback( VoltageCallbackFn voltageCallback )
{
	m_voltageCallback = voltageCallback;
}

//////////////////////////////////////////////////////////////////////////
// Message Processing
//////////////////////////////////////////////////////////////////////////

void BrainStemMessageProcessor::processHardwareMessage(vector<char> buffer)
{
    BRAIN_STEM_MSG eCommand = BRAIN_STEM_MSG::UNKNOWN;
    char messageKey = 0x00;

    if (buffer.size() > 0)
    {
        messageKey = buffer[0];
        eCommand = static_cast<BRAIN_STEM_MSG>(buffer[0]);
    }

	switch (eCommand)
	{
		case BRAIN_STEM_MSG::MESSAGE:
		{
			std::string strMessage( buffer.begin( ), buffer.end( ) );

			if( strMessage.find( "<MSG Error" ) != -1 )
			{
				ROS_ERROR_NAMED( "firmware", "Fatal Error: %s", strMessage.c_str( ) );
			}
			else
			{
				ROS_DEBUG_STREAM_NAMED( "firmware", "Message: <" << strMessage << ">" );
			}
            return;
		}

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
            return;
		}

		case BRAIN_STEM_MSG::HARDWARE_INFO:
		{
			HARDWARE_INFORMATION_DATA* pHardwareInfo = reinterpret_cast<HARDWARE_INFORMATION_DATA*>( buffer.data( ) );

			char* pszBrainStemVersion = (char*)buffer.data( ) + sizeof(HARDWARE_INFORMATION_DATA);

			std::string strBrainStemVersion( pszBrainStemVersion );

			m_hardwareInfoCallback( pHardwareInfo->uniqueId, pHardwareInfo->chassisGeneration,
				pHardwareInfo->brainstemHwVersion, strBrainStemVersion );
            return;
		}

		case BRAIN_STEM_MSG::OPERATIONAL_STATE:
		{
			OPERATIONAL_STATE_DATA* pOperationalState = reinterpret_cast<OPERATIONAL_STATE_DATA*>( buffer.data( ) );

			std::bitset<8> motionStatusSet( pOperationalState->motionStatus );
			std::bitset<8> failureStatusSet( pOperationalState->failureStatus );

			MOTION_STATUS_DATA motionStatusData;
			motionStatusData.frontEStop = motionStatusSet.test( MOTION_STATUS::FRONT_E_STOP );
			motionStatusData.backEStop = motionStatusSet.test( MOTION_STATUS::BACK_E_STOP );
			motionStatusData.wirelessEStop = motionStatusSet.test( MOTION_STATUS::WIRELESS_E_STOP );
			motionStatusData.bumpSensor = motionStatusSet.test( MOTION_STATUS::BUMP_SENSOR);
			motionStatusData.pause = motionStatusSet.test( MOTION_STATUS::PAUSE );
			motionStatusData.hardStop = motionStatusSet.test( MOTION_STATUS::HARD_STOP );

			FAILURE_STATUS_DATA failureStatusData;
			failureStatusData.safetyProcessorFailure = failureStatusSet.test( FAILURE_STATUS::SAFETY_PROCESSOR );
			failureStatusData.brainstemFailure = failureStatusSet.test( FAILURE_STATUS::BRAINSTEM );
			failureStatusData.brainTimeoutFailure = failureStatusSet.test( FAILURE_STATUS::BRAINSTEM_TIMEOUT );
			failureStatusData.rightMotorFailure = failureStatusSet.test( FAILURE_STATUS::RIGHT_MOTOR );
			failureStatusData.leftMotorFailure = failureStatusSet.test( FAILURE_STATUS::LEFT_MOTOR );

			m_operationalStateCallback( pOperationalState->upTime, motionStatusData,
				failureStatusData );
            return;
		}

		case BRAIN_STEM_MSG::SYSTEM_VOLTAGE:
		{
			VOLTAGE_DATA* pVoltage = reinterpret_cast<VOLTAGE_DATA*>( buffer.data( ) );

			m_voltageCallback( pVoltage->voltage );
            return;
		}
	}

    // Go through the registered message handlers and
    // communicate the data if the key matches
	ros::Time currentTime = ros::Time::now();
    for (auto handler : hwMessageHandlers_)
    {
        if (handler->isKeyMatching(messageKey))
        {
            handler->receiveData(currentTime, buffer);
            return;
        }
    }

    // If it arrives here, the message key is unknown
    ROS_ERROR_STREAM( "Unknown message from brainstem: " <<
        static_cast<int>(eCommand) << ", data: " << ToHex(buffer));
}

void BrainStemMessageProcessor::GetOperationalState( )
{
	ROS_DEBUG( "GetOperationalState" );

	COMMAND_DATA msg = { static_cast<uint8_t>( BRAIN_STEM_CMD::GET_OPERATIONAL_STATE) };

	// Get the operational state
	WriteToSerialPort( reinterpret_cast<char*>( &msg ), sizeof( msg ) );
}

void BrainStemMessageProcessor::GetHardwareInformation( )
{
	ROS_DEBUG( "GetHardwareInformation" );

	COMMAND_DATA msg = { static_cast<uint8_t>( BRAIN_STEM_CMD::GET_HARDWARE_INFO ) };

	// Get the hardware information (version, configuration, etc.)
	WriteToSerialPort( reinterpret_cast<char*>( &msg ), sizeof( msg ) );
}

void BrainStemMessageProcessor::SendPing( )
{
	uint8_t cMessage = static_cast<uint8_t>( BRAIN_STEM_CMD::PING );

	WriteToSerialPort( reinterpret_cast<char*>( &cMessage ), 1 );
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
		ROS_DEBUG_NAMED( "velocity", "Brainstem: SetVelocity: %f, %f", dfLinear, dfAngular );

		s_dfLinear = dfLinear;
		s_dfAngular = dfAngular;
	}

	// Send the velocity down to the motors
	WriteToSerialPort( reinterpret_cast<char*>( &msg ), sizeof( msg ) );
}

////////////////////////////////////////////////////////////////////////////
//// ROS Callbacks
////////////////////////////////////////////////////////////////////////////

void BrainStemMessageProcessor::processRosMessage(const string& strMessage)
{
	boost::char_separator<char> separator(";");

	boost::tokenizer<boost::char_separator<char>> messageListTokenizer( strMessage, separator );

	// Parse the list of semicolon delimited commands
	for( auto && strCommand : messageListTokenizer )
	{
		ROS_DEBUG( "Parse command: %s", strCommand.c_str( ) );

		// Parse the command by whitespace
		std::vector<std::string> vecParsed;

		boost::char_separator<char> spaceSeparator(" ");

		boost::tokenizer<boost::char_separator<char>> commandListTokenizer( strCommand, spaceSeparator );

		for( auto && strValue : commandListTokenizer )
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
					ROS_ERROR( "Bridge has invalid number of arguments for command %s: %s",
						strCommand.c_str( ), strMessage.c_str( ) );
				}
			}
			else
			{
				ROS_ERROR( "Invalid command %s", strCommand.c_str( ) );

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

void BrainStemMessageProcessor::SetMotionStatus( const std::bitset<8>& motionStatusSet, bool bSetValues )
{
	std::string strMotionStatus;
	strMotionStatus += motionStatusSet.test( MOTION_STATUS::FRONT_E_STOP ) ? "frontEStop, " : "";
	strMotionStatus += motionStatusSet.test( MOTION_STATUS::BACK_E_STOP ) ? "backEStop, " : "";
	strMotionStatus += motionStatusSet.test( MOTION_STATUS::WIRELESS_E_STOP ) ? "wirelessEStop, " : "";
	strMotionStatus += motionStatusSet.test( MOTION_STATUS::BUMP_SENSOR ) ? "bumpSensor, " : "";
	strMotionStatus += motionStatusSet.test( MOTION_STATUS::PAUSE ) ? "pause, " : "";
	strMotionStatus += motionStatusSet.test( MOTION_STATUS::HARD_STOP ) ? "hardStop, " : "";

	BRAIN_STEM_CMD command = bSetValues ? BRAIN_STEM_CMD::SET_MOTION_STATUS : BRAIN_STEM_CMD::CLEAR_MOTION_STATUS;

	SET_OPERATIONAL_STATE_DATA msg = { static_cast<uint8_t>( command ), static_cast<uint8_t>( motionStatusSet.to_ulong( ) ) };

	ROS_DEBUG( "%s motion status for %s", command == BRAIN_STEM_CMD::SET_MOTION_STATUS ? "Setting" : "Clearing", strMotionStatus.c_str( ) );

	WriteToSerialPort( reinterpret_cast<char*>( &msg ), sizeof(msg) );
}

void BrainStemMessageProcessor::OnHardStop( )
{
	ROS_DEBUG( "OnHardStop" );

	uint8_t cMessage = static_cast<uint8_t>( BRAIN_STEM_CMD::HARD_STOP );

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
		ROS_DEBUG( "OnUpdateLights (0x%x): %d=>%d", msg.cmd, msg.entitiy, msg.mode );

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

	Pause( strPaused == "ON" );
}

void BrainStemMessageProcessor::ClearMotionStatus( )
{
	ROS_DEBUG( "Clearing motion status." );

	std::bitset<8> clearSet;
	clearSet.set( MOTION_STATUS::WIRELESS_E_STOP, true );
	clearSet.set( MOTION_STATUS::BUMP_SENSOR, true );
	clearSet.set( MOTION_STATUS::HARD_STOP, true );

	SetMotionStatus( clearSet, false );
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

void BrainStemMessageProcessor::Pause( bool bPaused )
{
	std::bitset<8> pauseSet;
	pauseSet.set( MOTION_STATUS::PAUSE, true );

	SetMotionStatus( pauseSet, bPaused );
}

} /* namespace srs */
