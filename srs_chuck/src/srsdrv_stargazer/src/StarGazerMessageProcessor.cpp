/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#include <ros/ros.h>
#include <chrono>
#include <sstream>
#include <string>
#include <iostream>
#include <boost/tokenizer.hpp>

#include "StarGazerMessage.h"
#include "StarGazerMessageProcessor.h"
#include "StarGazerSerialIO.h"
#include "srslib_framework/platform/Thread.hpp"
#include "srslib_framework/io/SerialIO.hpp"

namespace srs
{

static const std::string FLASH_WRITE_DONE = "!ParameterUpdate";

StarGazerMessageProcessor::StarGazerMessageProcessor( std::shared_ptr<IO> pIO ) :
	m_readCallback( ),
	m_odometryCallback( ),
	m_pSerialIO( pIO ),
	m_lastTxMessage( ),
	m_txMessageQueue( ),
	m_lastTxTime( std::chrono::milliseconds( 0 ) ),
	m_odometryRegex( ),
	m_messageRegex( ),
	m_highrezclk( )
{
	// Protect against odd compiler versions (boost seems to fix the regex bugs in gcc 4.8.4)
	try
	{
		m_odometryRegex = boost::regex(
			"\\^[FIZ]([0-9]*)\\|([+-][0-9]*\\.[0-9]*)\\|([+-][0-9]*\\.[0-9]*)\\|([+-][0-9]*\\.[0-9]*)\\|(-?[0-9]*\\.[0-9]*)" );

		m_messageRegex = boost::regex( "\\$([^\\|]*)(?:\\|([^\\`]*))?" );
	}
	catch( const boost::regex_error& e )
	{
		ROS_ERROR( "regex: %s (%d)", e.what( ), e.code( ) );
	}
}

StarGazerMessageProcessor::~StarGazerMessageProcessor( )
{

}

void StarGazerMessageProcessor::SetOdometryCallback( OdometryCallbackFn odometryCallback )
{
	m_odometryCallback = odometryCallback;
}

void StarGazerMessageProcessor::SetReadCallback( ReadCallbackFn readCallback )
{
	m_readCallback = readCallback;
}

//////////////////////////////////////////////////////////////////////////
// StarGazer Commands
//////////////////////////////////////////////////////////////////////////

void StarGazerMessageProcessor::SendRawCommand( QueuedMessage msg )
{
	ROS_DEBUG_STREAM_NAMED( "StarGazer", "Rawsend: " << msg.sTxMsg << " Expecting " << msg.sRxMsg << " Timeout: " << msg.timeoutSec );

	m_lastTxTime = m_highrezclk.now( );

	m_lastTxMessage = msg.sRxMsg;

	std::vector<char> cmdVec( msg.sTxMsg.begin( ), msg.sTxMsg.end( ) );

	m_pSerialIO->Write( cmdVec );
}

void StarGazerMessageProcessor::BaseCommand(STAR_GAZER_MESSAGE_TYPES type, std::string cmd, std::string rxExpected = "", float timeoutSec = STARTGAZER_TIMEOUT)
{
	// Build the command
	std::string fullCmd( "" );
	fullCmd += STARGAZER_STX;
	fullCmd += (char) type;
	fullCmd += cmd;
	fullCmd += STARGAZER_RTX;

	// Assume we are expecting a return value if this is read or an ack otherwise
	if( rxExpected.size( ) == 0 )
	{
		if( type == STAR_GAZER_MESSAGE_TYPES::READ )
		{
			rxExpected += (char) STAR_GAZER_MESSAGE_TYPES::RETURN_VALUE;
		}
		else
		{
			rxExpected += (char) STAR_GAZER_MESSAGE_TYPES::ACK;
		}
		auto cmdtypeEnd = std::find( cmd.begin( ), cmd.end( ), STARGAZER_SEPERATOR );

		rxExpected += std::string( cmd.begin( ), cmdtypeEnd );
	}

	QueuedMessage msg;
	msg.sTxMsg = fullCmd;
	msg.sRxMsg = rxExpected;
	msg.timeoutSec = timeoutSec;

	m_txMessageQueue.push( msg );

	// If we are the only thing in the queue, that means that
	// there is no outstanding outgoing message.  We can just
	// send out the message
	if( m_txMessageQueue.size( ) == 1 )
	{
		SendRawCommand( msg );
	}

	ROS_DEBUG_STREAM_NAMED( "StarGazer", fullCmd );
}

void StarGazerMessageProcessor::BaseWriteCommand( std::string cmd )
{
	BaseCommand( STAR_GAZER_MESSAGE_TYPES::WRITE, cmd );
}

void StarGazerMessageProcessor::BaseWriteCommand( std::string cmd, std::string arg1 )
{
	std::string compoundCommand( cmd );
	compoundCommand += STARGAZER_SEPERATOR;
	compoundCommand += arg1;
	BaseCommand( STAR_GAZER_MESSAGE_TYPES::WRITE, compoundCommand );
}

void StarGazerMessageProcessor::BaseWriteCommand( std::string cmd, int arg1 )
{
	BaseWriteCommand( cmd, std::to_string( arg1 ) );
}

void StarGazerMessageProcessor::BaseReadCommand( std::string cmd )
{
	BaseCommand( STAR_GAZER_MESSAGE_TYPES::READ, cmd );
}

void StarGazerMessageProcessor::CalcStop( )
{
	BaseWriteCommand( "CalcStop" );
}

void StarGazerMessageProcessor::CalcStart( )
{
	BaseWriteCommand( "CalcStart" );
}

void StarGazerMessageProcessor::SetMarkType( STAR_GAZER_LANDMARK_TYPES type )
{
	std::string cmd( "MarkType" );

	std::string arg1( "" );

	switch( type )
	{
		case STAR_GAZER_LANDMARK_TYPES::HLD1S:
		{
			arg1 += "HLD1S";
		}
		break;

		case STAR_GAZER_LANDMARK_TYPES::HLD1L:
		{
			arg1 += "HLD1L";
		}
		break;

		case STAR_GAZER_LANDMARK_TYPES::HLD2S:
		{
			arg1 += "HLD2S";
		}
		break;

		case STAR_GAZER_LANDMARK_TYPES::HLD2L:
		{
			arg1 += "HLD2L";
		}
		break;

		case STAR_GAZER_LANDMARK_TYPES::HLD3S:
		{
			arg1 += "HLD3S";
		}
		break;

		case STAR_GAZER_LANDMARK_TYPES::HLD3L:
		{
			arg1 += "HLD3L";
		}
		break;

		default:
		{
			// Throw error
			throw std::runtime_error( "unknown Marktype" );
		}
	}

	BaseWriteCommand( cmd, arg1 );
}

void StarGazerMessageProcessor::GetVersion( )
{
	BaseReadCommand( "Version" );
}

void StarGazerMessageProcessor::HightFix(bool fixHeight)
{
	BaseWriteCommand("HeightFix", std::string(fixHeight? "Yes" : "No"));
}

void StarGazerMessageProcessor::HeightCalc( )
{
	// This is going to be written back to flash, so we need to wait for that for confirmation
	// The calculation of height usually takes ~10sec, so we give a 15sec margin before retry
	BaseCommand( STAR_GAZER_MESSAGE_TYPES::WRITE, "HeightCalc", FLASH_WRITE_DONE, 15 );
}

void StarGazerMessageProcessor::SetMarkHeight( int height_mm )
{
	BaseWriteCommand( "MarkHeight", height_mm );
}

void StarGazerMessageProcessor::SetEnd( )
{
	// This writes parameters to flash and commits them.  It usually takes ~3 sec
	// to write to flash, so we give 4 seconds before a retry.
	BaseCommand( STAR_GAZER_MESSAGE_TYPES::WRITE, "SetEnd", FLASH_WRITE_DONE, 10 );
}

void StarGazerMessageProcessor::HardReset( )
{
	BaseWriteCommand( "Reset" );
}

void StarGazerMessageProcessor::PumpMessageProcessor( )
{
	std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(
		m_highrezclk.now( ) - m_lastTxTime );

	if( m_lastTxMessage == "!ParameterUpdate" )
	{
		ROS_DEBUG_STREAM_NAMED( "StarGazer", "Expecting " << m_lastAck << " took " << time_span.count( ) << " seconds. " );
	}

	if( m_lastTxMessage == m_lastAck )
	{
		ROS_DEBUG_STREAM_NAMED( "StarGazer", "Got Ack for " << m_lastAck << " took " << time_span.count( ) << " seconds. " );

		// Make sure we consume the ACK so that we don't get here again
		m_lastTxMessage = "";

		// Remove the message we have confirmed
		m_txMessageQueue.pop( );

		// If there is another message to send, then send it.  Otherwise do nothing
		if( m_txMessageQueue.size( ) > 0 )
		{
			ROS_DEBUG_STREAM_NAMED( "StarGazer", "Sending next message " << m_txMessageQueue.front( ).sTxMsg );

			SendRawCommand( m_txMessageQueue.front( ) );
		}
		else
		{
			ROS_DEBUG_STREAM_NAMED( "StarGazer", "Message Queue Empty" );
		}
	}
	// If we have timed out - resend the message
	else if( m_lastTxMessage.size( ) > 0 && time_span.count( ) > m_txMessageQueue.front( ).timeoutSec )
	{
		ROS_DEBUG_STREAM_NAMED("StarGazerMessageProcessor", "ACK timeout - retransmitting " << m_txMessageQueue.front( ).sTxMsg );

		SendRawCommand( m_txMessageQueue.front( ) );
	}
}

//////////////////////////////////////////////////////////////////////////
// StarGazer Callbacks
//////////////////////////////////////////////////////////////////////////

void StarGazerMessageProcessor::ProcessStarGazerMessage( std::vector<char> msgBuffer )
{
	bool printMessage = false;

	STAR_GAZER_MESSAGE_TYPES command = (STAR_GAZER_MESSAGE_TYPES) msgBuffer[0];

	auto typeStart = msgBuffer.begin( ) + 1;
	auto typeEnd = std::find( typeStart, msgBuffer.end( ), STARGAZER_SEPERATOR );

	std::string typeStr( typeStart, typeEnd );

	boost::smatch regexMatch;
	std::string regexString = std::string( msgBuffer.begin( ), msgBuffer.end( ) );

	switch( command )
	{
		case STAR_GAZER_MESSAGE_TYPES::POSE:
		{
			if( boost::regex_match( regexString, regexMatch, m_odometryRegex ) )
			{
				if( regexMatch.size( ) != 6 )
				{
					ROS_DEBUG_STREAM_NAMED( "StarGazerMessageProcessor", "Odometry with only " <<
						regexMatch.size( ) << " fields: " << std::string( msgBuffer.begin( ), msgBuffer.end( ) ) );
				}
				else if( m_odometryCallback )
				{
					try
					{
						int tagID = std::stoul( regexMatch[1] );

						float angle = std::stof( regexMatch[2] );

						float x = std::stof( regexMatch[3] );

						float y = std::stof( regexMatch[4] );

						float z = std::stof( regexMatch[5] );

						m_odometryCallback( tagID, x, y, z, angle );
					}
					catch( const std::invalid_argument& )
					{
						ROS_ERROR_STREAM_NAMED( "StarGazerMessageProcessor", "Odometry has invalid parameter" );
					}
					catch( const std::out_of_range& )
					{
						ROS_ERROR_STREAM_NAMED( "StarGazerMessageProcessor", "Odometry has parameter out of range" );
					}
				}
				else
				{
					ROS_ERROR_STREAM_NAMED( "StarGazerMessageProcessor",
						"Serial port data read but no callback specified!" );
				}
			}
			else
			{
				ROS_DEBUG_STREAM_NAMED( "StarGazerMessageProcessor", "Odometry with bad format: " <<
					std::string( msgBuffer.begin( ), msgBuffer.end( ) ) );
			}
		}
		break;

		case STAR_GAZER_MESSAGE_TYPES::MESSAGE:
		{
			ROS_DEBUG_STREAM_NAMED( "StarGazerMessageProcessor", typeStr );
		}
		break;

		case STAR_GAZER_MESSAGE_TYPES::ACK:
		{
			m_lastAck.clear( );
			m_lastAck += (char) command;
			m_lastAck += typeStr;

			ROS_ERROR_STREAM_NAMED( "StarGazerMessageProcessor", "Got command " << m_lastAck << " Expectd: " << m_lastTxMessage );
		}
		break;

		case STAR_GAZER_MESSAGE_TYPES::RETURN_VALUE:
		{
			m_lastAck.clear( );
			m_lastAck += (char) command;
			m_lastAck += typeStr;

			if( boost::regex_match( regexString, regexMatch, m_messageRegex ) )
			{
				if( regexMatch.size( ) != 3 )
				{
					ROS_DEBUG_STREAM_NAMED( "StarGazerMessageProcessor", "Return value with only "
						<< regexMatch.size( ) << " fields: " <<
						std::string( msgBuffer.begin( ), msgBuffer.end( ) ) );
				}
				else if( m_readCallback )
				{
					std::string param( "Not Implemented" );
					m_readCallback( typeStr, regexMatch[2] );
				}
				else
				{
					ROS_ERROR_STREAM_NAMED( "StarGazerMessageProcessor",
						"Serial port data read but no callback specified!\n" );
					printMessage = true;
				}
			}
			else
			{
				ROS_DEBUG_STREAM_NAMED( "StarGazerMessageProcessor", "Return value with bad format: " <<
					std::string( msgBuffer.begin( ), msgBuffer.end( ) ) );
				printMessage = true;

			}
		}
		break;

		default:
		{
			ROS_ERROR_STREAM_NAMED( "StarGazerMessageProcessor", "Unknown command type " << (char)command );

			printMessage = true;
		}
		break;
	}

	if( printMessage )
	{
		ROS_DEBUG_STREAM_NAMED( "StarGazerMessageProcessor", "Raw Message: \"" << std::string( msgBuffer.begin( ), msgBuffer.end( ) ) << "\"" );
	}
}

//////////////////////////////////////////////////////////////////////////
// Helper Methods
//////////////////////////////////////////////////////////////////////////

// TODO: Move down to base class
void StarGazerMessageProcessor::WriteToSerialPort( char* pszData, std::size_t dwSize )
{
	if( m_pSerialIO->IsOpen( ) )
	{
		m_pSerialIO->Write( std::vector<char>( pszData, pszData + dwSize ) );
	}
	else
	{
		ROS_ERROR_THROTTLE_NAMED( 60, "StarGazer",
			"Attempt to write to the serial port, but the serial port is not open!" );
	}
}

} /* namespace srs */
