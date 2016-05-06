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
#include "srslib_framework/utils/Thread.hpp"
#include "srslib_framework/io/SerialIO.hpp"

namespace srs
{

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
			"~\\^I([0-9]*)\\|([+-][0-9]*\\.[0-9]*)\\|([+-][0-9]*\\.[0-9]*)\\|([+-][0-9]*\\.[0-9]*)\\|(-?[0-9]*\\.[0-9]*)" );

		m_messageRegex = boost::regex( "~\\$([^\\|]*)(?:\\|([^\\`]*))?" );
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

void StarGazerMessageProcessor::SendRawCommand( std::string fullCmd )
{
	ROS_DEBUG_STREAM_NAMED( "StarGazer", "Rawsend: " << fullCmd );

	m_lastTxTime = m_highrezclk.now( );

	auto typeStart = fullCmd.begin( ) + 2;
	auto typeEndField = std::find( typeStart, fullCmd.end( ), StarGazer_SEPERATOR );
	auto typeEndMsg = std::find( typeStart, fullCmd.end( ), StarGazer_RTX );

	// The command ends at the first instance of message end or a separator
	if( typeEndField < typeEndMsg )
	{
		typeEndMsg = typeEndField;
	}

	m_lastTxMessage = std::string( typeStart, typeEndMsg );

	std::vector<char> cmdVec( fullCmd.begin( ), fullCmd.end( ) );

	m_pSerialIO->Write( cmdVec );
}

void StarGazerMessageProcessor::BaseCommand( STAR_GAZER_MESSAGE_TYPES type, std::string cmd )
{
	// Build the command
	std::string fullCmd( "" );
	fullCmd += StarGazer_STX;
	fullCmd += (char) type;
	fullCmd += cmd;
	fullCmd += StarGazer_RTX;

	m_txMessageQueue.push( fullCmd );

	// If we are the only thing in the queue, that means that
	// there is no outstanding outgoing message.  We can just
	// send out the message
	if( m_txMessageQueue.size( ) == 1 )
	{
		SendRawCommand( fullCmd );
	}
}

void StarGazerMessageProcessor::BaseWriteCommand( std::string cmd )
{
	BaseCommand( STAR_GAZER_MESSAGE_TYPES::WRITE, cmd );
}

void StarGazerMessageProcessor::BaseWriteCommand( std::string cmd, std::string arg1 )
{
	std::string compoundCommand( cmd );
	compoundCommand += StarGazer_SEPERATOR;
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

void StarGazerMessageProcessor::HeightCalc( )
{
	BaseWriteCommand( "HeightCalc" );
}

void StarGazerMessageProcessor::SetMarkHeight( int height_mm )
{
	BaseWriteCommand( "MarkHeight", height_mm );
}

void StarGazerMessageProcessor::SetEnd( )
{
	BaseWriteCommand( "SetEnd" );
}

void StarGazerMessageProcessor::HardReset( )
{
	BaseWriteCommand( "Reset" );
}

void StarGazerMessageProcessor::PumpMessageProcessor( )
{
	std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(
		m_highrezclk.now( ) - m_lastTxTime );

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
			ROS_DEBUG_STREAM_NAMED( "StarGazer", "Sending next message " << m_txMessageQueue.front( ) );

			SendRawCommand( m_txMessageQueue.front( ) );
		}
		else
		{
			ROS_DEBUG_STREAM_NAMED( "StarGazer", "Message Queue Empty" );
		}
	}
	// If we have timed out - resend the message
	else if( m_lastTxMessage.size( ) > 0 && time_span.count( ) > STARTGAZER_TIMEOUT )
	{
		ROS_DEBUG_STREAM_NAMED( "StarGazer", "No Ack in " << time_span.count( ) << ", retransmitting "
			<< m_txMessageQueue.front( ) << std::endl );

		SendRawCommand( m_txMessageQueue.front( ) );
	}
}

//////////////////////////////////////////////////////////////////////////
// StarGazer Callbacks
//////////////////////////////////////////////////////////////////////////

void StarGazerMessageProcessor::ProcessStarGazerMessage( std::vector<char> msgBuffer )
{
	bool printMessage = false;

	STAR_GAZER_MESSAGE_TYPES command = (STAR_GAZER_MESSAGE_TYPES) msgBuffer[1];

	auto typeStart = msgBuffer.begin( ) + 2;
	auto typeEnd = std::find( typeStart, msgBuffer.end( ), StarGazer_SEPERATOR );

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
			m_lastAck = typeStr;
		}
		break;

		case STAR_GAZER_MESSAGE_TYPES::RETURN_VALUE:
		{
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
