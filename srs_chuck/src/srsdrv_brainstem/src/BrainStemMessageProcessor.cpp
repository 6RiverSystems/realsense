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

#include "../include/srsdrv_brainstem/BrainStemMessages.hpp"

namespace srs {

using namespace ros;

BrainStemMessageProcessor::BrainStemMessageProcessor( std::shared_ptr<IO> pIO ) :
	m_pIO( pIO ),
	isConnected_(false),
	hasValidHardareInfo_(false),
	lastHardareInfoRequestTime_(),
	hasValidOperationalState_(false),
	lastOperationalStateRequestTime_(),
	connectedChannel_( ),
    soundHandler_(this),
	setMotionStateHandler_(this),
	pingHandler_(this),
	odometryRpmChannel_(),
	hardwareInfoChannel_(),
	operationalStateChannel_(),
	powerStateChannel_(),
	logHandler_(),
	odometryRpmHandler_(odometryRpmChannel_),
	hardwareInfoHandler_(hardwareInfoChannel_),
	operationalStateHandler_(operationalStateChannel_),
	powerStateHandler_(powerStateChannel_),
	buttonPressedHandler_(buttonPressedChannel_)
{
    hwMessageHandlers_[odometryRpmHandler_.getKey()] = &odometryRpmHandler_;
    hwMessageHandlers_[hardwareInfoHandler_.getKey()] = &hardwareInfoHandler_;
    hwMessageHandlers_[operationalStateHandler_.getKey()] = &operationalStateHandler_;
    hwMessageHandlers_[powerStateHandler_.getKey()] = &powerStateHandler_;
    hwMessageHandlers_[buttonPressedHandler_.getKey()] = &buttonPressedHandler_;
    hwMessageHandlers_[logHandler_.getKey()] = &logHandler_;
}

BrainStemMessageProcessor::~BrainStemMessageProcessor( )
{

}

//////////////////////////////////////////////////////////////////////////
// Message Processing
//////////////////////////////////////////////////////////////////////////

void BrainStemMessageProcessor::processHardwareMessage(vector<char> buffer)
{
    BRAIN_STEM_MSG eCommand = BRAIN_STEM_MSG::UNKNOWN;

    if (buffer.size() > 0)
    {
        eCommand = static_cast<BRAIN_STEM_MSG>(buffer[0]);

		ros::Time currentTime = ros::Time::now();

		// Go through the registered message handlers and communicate the data if the key matches
		HwMessageHandlerMapType::iterator handler = hwMessageHandlers_.find(eCommand);
		if (handler != hwMessageHandlers_.end())
		{
			try
			{
				handler->second->receiveData(currentTime, buffer);

				getHardwareInfo(currentTime);

				getOperationalState(currentTime);
			}
			catch(std::runtime_error& error)
			{
				// If it arrives here, the message failed to parse
				ROS_ERROR_STREAM( "Message from brainstem malformed: " <<
					static_cast<char>(eCommand) << ", data: " << ToHex(buffer) << ", exception: " << error.what());
			}
		}
		else
		{
			if (isSetupComplete() &&
				eCommand != BRAIN_STEM_MSG::SYSTEM_VOLTAGE  &&
				eCommand != BRAIN_STEM_MSG::SENSOR_FRAME &&
				eCommand != BRAIN_STEM_MSG::MESSAGE)
			{
			    ROS_ERROR_STREAM( "Unknown message from brainstem: " <<
			        static_cast<int>(eCommand) << ", data: " << ToHex(buffer));
			}
		}
    }
}

void BrainStemMessageProcessor::getOperationalState( )
{
	ROS_DEBUG( "GetOperationalState" );

	CommandData msg = { static_cast<uint8_t>( BRAIN_STEM_CMD::GET_OPERATIONAL_STATE) };

	// Get the operational state
	WriteToSerialPort( reinterpret_cast<char*>( &msg ), sizeof( msg ) );
}

void BrainStemMessageProcessor::getHardwareInformation( )
{
	ROS_DEBUG( "GetHardwareInformation" );

	CommandData msg = { static_cast<uint8_t>( BRAIN_STEM_CMD::GET_HARDWARE_INFO ) };

	// Get the hardware information (version, configuration, etc.)
	WriteToSerialPort( reinterpret_cast<char*>( &msg ), sizeof( msg ) );
}

void BrainStemMessageProcessor::shutdown( )
{
	uint8_t cMessage = static_cast<uint8_t>( BRAIN_STEM_CMD::SHUTDOWN );

	WriteToSerialPort( reinterpret_cast<char*>( &cMessage ), 1 );
}

//////////////////////////////////////////////////////////////////////////
// Bridge Callbacks
//////////////////////////////////////////////////////////////////////////

void BrainStemMessageProcessor::setConnected( bool isConnected )
{
	isConnected_ = isConnected;

	checkSetupComplete();
}

void BrainStemMessageProcessor::getHardwareInfo(const ros::Time& now)
{
	if (!hasValidHardareInfo_)
	{
		ros::Duration duration = now - lastHardareInfoRequestTime_;
		if (duration.toSec() > 1.0f)
		{
			// Send another request
			getHardwareInformation();
		}

		hasValidHardareInfo_ = true;

		checkSetupComplete();
	}
}

void BrainStemMessageProcessor::getOperationalState(const ros::Time& now)
{
	if (!hasValidOperationalState_)
	{
		ros::Duration duration = now - lastOperationalStateRequestTime_;
		if (duration.toSec() > 1.0f)
		{
			// Send another request
			getOperationalState();
		}

		hasValidOperationalState_ = true;

		checkSetupComplete();
	}
}

bool BrainStemMessageProcessor::isSetupComplete() const
{
	return isConnected_ &&
		hasValidHardareInfo_ &&
		hasValidOperationalState_;
}

void BrainStemMessageProcessor::checkSetupComplete()
{
	bool setupComplete = isSetupComplete();
	if (setupComplete)
	{
		ROS_INFO( "Setup complete (received hardware info and operational state message." );

		m_pIO->SetSynced(true);
	}

	connectedChannel_.publish(setupComplete);
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
