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

static const std::string FLASH_WRITE_DONE = "ParameterUpdate";

StarGazerMessageProcessor::StarGazerMessageProcessor( std::shared_ptr<IO> pIO ) :
	m_readCallback( ),
	m_odometryCallback( ),
	m_pSerialIO( pIO ),
	m_queueOutgoingMessages( ),
	m_mapPendingResponses( ),
	m_odometryRegex( ),
	m_messageRegex( ),
	m_highrezclk( ),
	m_bIsStarted( false )
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

void StarGazerMessageProcessor::ResetState( )
{
	std::queue<QueuedMessage> empty;
	std::swap( m_queueOutgoingMessages, empty );

	m_mapPendingResponses.clear( );

	m_bIsStarted = false;
}

//////////////////////////////////////////////////////////////////////////
// StarGazer Commands
//////////////////////////////////////////////////////////////////////////

void StarGazerMessageProcessor::SendNextMessage( )
{
	if( m_queueOutgoingMessages.size( ) )
	{
		m_queueOutgoingMessages.pop( );

		if( m_queueOutgoingMessages.size( ) )
		{
			SendRawCommand( m_queueOutgoingMessages.front( ) );
		}
	}
}

void StarGazerMessageProcessor::SendRawCommand( QueuedMessage msg )
{
	if( m_mapPendingResponses.find( msg.command ) == m_mapPendingResponses.end( ) )
	{
		ROS_DEBUG_STREAM_NAMED( "StarGazer", "Rawsend: " << msg.command <<
			", Timeout: " << msg.timeout.count( ) );

		m_mapPendingResponses[msg.expectedAck1] = { m_highrezclk.now( ), msg };

		std::vector<char> cmdVec;
		cmdVec.push_back( (char)msg.type );
		cmdVec.insert( cmdVec.end( ), msg.command.begin( ), msg.command.end( ) );

		m_pSerialIO->Write( cmdVec );
	}
	else
	{
		ROS_DEBUG_STREAM_NAMED( "StarGazer", "Attempt to write duplicate message (already pending): " << msg.command );
	}
}

void StarGazerMessageProcessor::BaseCommand( STAR_GAZER_MESSAGE_TYPES type, std::string command,
	std::string additionalAck = "", std::chrono::microseconds timeout = STARTGAZER_TIMEOUT )
{
	auto cmdtypeEnd = std::find( command.begin( ), command.end( ), STARGAZER_SEPERATOR );

	std::string expectedAck = std::string( command.begin( ), cmdtypeEnd );

	QueuedMessage msg;
	msg.type = type;
	msg.command = command;
	msg.expectedAck1 = expectedAck;
	msg.expectedAck2 = additionalAck;
	msg.timeout = timeout;

	m_queueOutgoingMessages.push( msg );

	// If we are the only thing in the queue, that means that
	// there is no outstanding outgoing message.  We can just
	// send out the message
	if( m_queueOutgoingMessages.size( ) == 1 )
	{
		SendRawCommand( m_queueOutgoingMessages.front( ) );
	}
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
	BaseWriteCommand( STARGAZER_STOP );
}

void StarGazerMessageProcessor::CalcStart( )
{
	BaseWriteCommand( STARGAZER_START );
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
	BaseCommand( STAR_GAZER_MESSAGE_TYPES::WRITE, "HeightCalc", FLASH_WRITE_DONE, std::chrono::seconds( 15 ) );
}

void StarGazerMessageProcessor::SetMarkHeight( int height_mm )
{
	BaseWriteCommand( "MarkHeight", height_mm );
}

void StarGazerMessageProcessor::SetEnd( )
{
	// This writes parameters to flash and commits them.  It usually takes ~3 sec
	// to write to flash, so we give 4 seconds before a retry.
	BaseCommand( STAR_GAZER_MESSAGE_TYPES::WRITE, "SetEnd", FLASH_WRITE_DONE, std::chrono::seconds( 10 ) );
}

void StarGazerMessageProcessor::HardReset( )
{
	BaseWriteCommand( "Reset" );
}

void StarGazerMessageProcessor::PumpMessageProcessor( )
{
	auto now = m_highrezclk.now( );

	std::map<std::string, PendingAck> mapExpiredItems;

	for( auto pendingAck : m_mapPendingResponses )
	{
		if( (now - pendingAck.second.timeSent) > pendingAck.second.msg.timeout )
		{
			mapExpiredItems[pendingAck.first] = pendingAck.second;
		}
	}

	for( auto expiredMsg : mapExpiredItems )
	{
		m_mapPendingResponses.erase( expiredMsg.first );

		if( expiredMsg.first == expiredMsg.second.msg.expectedAck2 )
		{
			// Assume that we have already saved the value since no ack is received
		}
		else
		{
			ROS_DEBUG_STREAM_NAMED("StarGazerMessageProcessor", "ACK timeout - retransmitting " << expiredMsg.second.msg.command );

			SendRawCommand( expiredMsg.second.msg );
		}
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
			if( m_bIsStarted )
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
			else
			{
				ROS_DEBUG_STREAM_THROTTLE_NAMED( 0.5, "StarGazerMessageProcessor", "Odometry info recieved before start, ignoring." );
			}
		}
		break;

		case STAR_GAZER_MESSAGE_TYPES::MESSAGE:
		{
			ROS_DEBUG_STREAM_NAMED( "StarGazer write duplicate messageMessageProcessor", typeStr );
		}
		break;

		case STAR_GAZER_MESSAGE_TYPES::ACK:
		{
			auto iter = m_mapPendingResponses.find( typeStr );

			if( iter != m_mapPendingResponses.end( ) )
			{
				auto pendingMessage = iter->second;

				ROS_DEBUG_STREAM_NAMED( "StarGazerMessageProcessor", "Got ack for: " << typeStr );

				// If we are expecting an additional ack then add it to the pending message list
				if( pendingMessage.msg.expectedAck2.size( ) &&
					pendingMessage.msg.expectedAck1 == typeStr )
				{
					ROS_DEBUG_STREAM_NAMED( "StarGazerMessageProcessor", "Adding additional expected ack: " << pendingMessage.msg.expectedAck2 );

					m_mapPendingResponses[pendingMessage.msg.expectedAck2] = pendingMessage;

					m_mapPendingResponses.erase( iter );
				}
				// If we are expecting read data, then wait for RETURN_VALUE message
				else if( pendingMessage.msg.type != STAR_GAZER_MESSAGE_TYPES::READ )
				{
					if( pendingMessage.msg.command == STARGAZER_START )
					{
						m_bIsStarted = true;
					}
					else if( pendingMessage.msg.command == STARGAZER_STOP )
					{
						m_bIsStarted = false;
					}

					// Erase the pending ack from our expected responses
					m_mapPendingResponses.erase( iter );

					SendNextMessage( );
				}
			}
			else
			{
				ROS_ERROR_STREAM_NAMED( "StarGazerMessageProcessor", "Got ack for unknown command: " << typeStr );
			}
		}
		break;

		case STAR_GAZER_MESSAGE_TYPES::RETURN_VALUE:
		{
			auto iter = m_mapPendingResponses.find( typeStr );

			if( iter != m_mapPendingResponses.end( ) )
			{
				m_mapPendingResponses.erase( iter );

				ROS_DEBUG_STREAM_NAMED( "StarGazerMessageProcessor", "Got return value for: " << typeStr );

				SendNextMessage( );
			}
			else
			{
				ROS_ERROR_STREAM_NAMED( "StarGazerMessageProcessor", "Got ack for unknown command: " << typeStr );
			}

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
