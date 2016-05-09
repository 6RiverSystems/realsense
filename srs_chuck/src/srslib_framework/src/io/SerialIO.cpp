/*
 * SerialIO.cpp
 *
 *  Created on: Apr 4, 2016
 *      Author: dan
 */

#include <srslib_framework/io/SerialIO.hpp>
#include <srslib_framework/utils/Logging.hpp>
#include <srslib_framework/platform/Thread.hpp>
#include <thread>
#include <ros/ros.h>

namespace srs {

SerialIO::SerialIO( ) :
	m_Thread( ),
	m_oWork( m_IOService ),
	m_SerialPort( m_IOService ),
	m_bIsSerialOpen( false ),
	m_strSerialPort( ),
	m_fRetryTimeout( 1.0 ),
	m_bIsWriting( false ),
	m_ReadBuffer( 1024 ),
	m_writeData( ),
	m_readData( ),
	m_readCallback( ),
	m_bGenerateCRC( false ),
	m_bIncludeLength( false ),
	m_bHasLeading( false ),
	m_cLeading( 0 ),
	m_bHasTerminating( false ),
	m_cTerminating( 0 ),
	m_bHasEscape( false ),
	m_cEscape( 0 ),
	m_setCharsToEscape( ),
	m_firstByteDelay( ),
	m_interByteDelay( )
{
	m_oTimer.reset( new boost::asio::deadline_timer( m_IOService, boost::posix_time::seconds( 5 ) ) );

	// Spin up the thread
	m_Thread.reset( new std::thread( [&]()
		{
			m_IOService.run( );
		} ) );
}

SerialIO::~SerialIO( )
{
	Close( );

	// Stop the service (and the thread)
	m_IOService.stop( );

	// Clean up the thread
	m_Thread->join( );
}

void SerialIO::Open( const char* pszName, std::function<void(std::vector<char>)> readCallback )
{
	m_readCallback = readCallback;

	OnCheckSerialPort( true );
}

bool SerialIO::IsOpen( ) const
{
	return m_SerialPort.is_open( );
}

void SerialIO::Close( )
{
	m_SerialPort.close( );
}

////////////////////////////////////////////////////////////////////////////////////////////
// Serial IO Configuration
////////////////////////////////////////////////////////////////////////////////////////////

void SerialIO::SetRetryTimeout( float fRetryTimeout )
{
	m_fRetryTimeout = fRetryTimeout;
}

void SerialIO::EnableCRC( bool bGenerateCRC )
{
	m_fRetryTimeout = bGenerateCRC;
}

void SerialIO::SetIncludeLength( bool bIncludeLength )
{
	m_bIncludeLength = bIncludeLength;
}

void SerialIO::SetLeadingCharacter( char cLeading )
{
	m_cLeading = cLeading;

	m_bHasLeading = true;
}

void SerialIO::SetTerminatingCharacter( char cTerminating )
{
	m_cTerminating = cTerminating;

	m_bHasTerminating = true;
}

void SerialIO::SetEscapeCharacter( char cEscape )
{
	m_cEscape = cEscape;

	m_bHasEscape = true;
}

void SerialIO::SetEscapeCharacters( std::set<char> vecCharsToEscape )
{
	m_setCharsToEscape = vecCharsToEscape;
}

void SerialIO::SetFirstByteDelay( std::chrono::microseconds firstByteDelay )
{
	m_firstByteDelay = firstByteDelay;
}

void SerialIO::SetByteDelay( std::chrono::microseconds byteDelay )
{
	m_byteDelay = byteDelay;
}

void SerialIO::Write( const std::vector<char>& buffer )
{
	if( IsOpen( ) )
	{
		// Do all data operations in serial processing thread
		m_IOService.post( std::bind( &SerialIO::WriteInSerialThread, this, buffer, false ) );
	}
	else
	{
		// Throw error
		throw std::runtime_error( "port not open" );
	}
}

void SerialIO::WriteRaw( const std::vector<char>& buffer )
{
	if( IsOpen( ) )
	{
		// Do all data operations in serial processing thread
		m_IOService.post( std::bind( &SerialIO::WriteInSerialThread, this, buffer, true ) );
	}
	else
	{
		// Throw error
		throw std::runtime_error( "port not open" );
	}
}

void SerialIO::WriteInSerialThread( std::vector<char> buffer, bool bIsRaw )
{
	uint32_t dwStartIndex = m_writeData.size( );

	if( !bIsRaw )
	{
		std::vector<char> payload;

		// Add length
		if( m_bIncludeLength )
		{
			m_writeData.push_back( (uint8_t)buffer.size( ) );
		}
	}

	// Add payload data
	m_writeData.insert( m_writeData.end( ), buffer.begin( ), buffer.end( ) );

	if( !bIsRaw && ( m_bGenerateCRC || m_setCharsToEscape.size( ) ) )
	{
		uint8_t cCRC = 0;

		// Escape characters and generate CRC
		for( std::size_t dwIndex = dwStartIndex; dwIndex != m_writeData.size( ); dwIndex++ )
		{
			cCRC += m_writeData[dwIndex];

			// Escape the character
			if( m_setCharsToEscape.find( m_writeData[dwIndex] ) != m_setCharsToEscape.end( ) )
			{
				m_writeData.insert( m_writeData.begin( ) + dwIndex, m_cEscape );

				// Skip the escaped character
				dwIndex++;
			}
		};

		// Generate CRC
		if( m_bGenerateCRC )
		{
			// Escape the character
			if( m_setCharsToEscape.find( -cCRC ) != m_setCharsToEscape.end( ) )
			{
				m_writeData.push_back( m_cEscape );
			}

			m_writeData.push_back( -cCRC );
		}

		// Add terminating character
		if( m_cTerminating )
		{
			m_writeData.push_back( m_cTerminating );
		}
	}

	// If we are already trying to write, then wait until we recieve the callback
	if( !m_bIsWriting )
	{
		m_bIsWriting = true;

		m_interByteDelay = m_firstByteDelay;

		// If we have a first byte delay only write one byte
		size_t writeSize = (m_interByteDelay == std::chrono::microseconds( )) ? m_writeData.size( ) : 1 ;

		m_SerialPort.async_write_some( boost::asio::buffer( m_writeData.data( ), writeSize ),
		    std::bind( &SerialIO::OnWriteComplete, this, std::placeholders::_1, std::placeholders::_2 ) );
	}
}

void SerialIO::StartAsyncRead( )
{
	m_SerialPort.async_read_some( boost::asio::buffer( m_ReadBuffer, m_ReadBuffer.size( ) ),
    	std::bind( &SerialIO::OnReadComplete, this, std::placeholders::_1, std::placeholders::_2 ) );
}

void SerialIO::OnWriteComplete( const boost::system::error_code& error, std::size_t size )
{
	ROS_DEBUG_STREAM_NAMED( "SerialIO", "Write: " <<
		ToHex( std::vector<char>( m_writeData.begin( ), m_writeData.end( ) + size ) ) );

	std::vector<char>( m_writeData.begin( ) + size, m_writeData.end( ) ).swap( m_writeData );

	if( m_writeData.begin( ) != m_writeData.end( ) )
	{
		std::this_thread::sleep_for( m_interByteDelay );

		m_SerialPort.async_write_some( boost::asio::buffer( m_writeData.data( ), m_writeData.size( ) ),
			std::bind( &SerialIO::OnWriteComplete, this, std::placeholders::_1, std::placeholders::_2 ) );

		m_interByteDelay = m_byteDelay;
	}
	else
	{
		m_bIsWriting = false;
	}
}

// TODO: Write all unit tests for cases which bit us the last go round
void SerialIO::OnReadComplete( const boost::system::error_code& error, std::size_t size )
{
	if( !error )
	{
		// Unescape the buffer if necessary (TODO: Fix edge case where last character is escape
		if( m_bHasEscape )
		{
			bool bIsEscaped = false;

			for( auto iter = m_ReadBuffer.begin( ); iter < m_ReadBuffer.begin( ) + size; iter++ )
			{
				if( *iter == m_cEscape )
				{
					if( bIsEscaped )
					{
						m_readData.push_back( *iter );

						bIsEscaped = false;
					}
					else
					{
						bIsEscaped = true;
					}
				}
				else if( bIsEscaped )
				{
					m_readData.push_back( *iter );

					bIsEscaped = false;
				}
			}
		}
		else
		{
			m_readData.insert( m_readData.end( ), m_ReadBuffer.begin( ), m_ReadBuffer.begin( ) + size );
		}

		auto msgStart = m_bHasLeading ? std::find( m_readData.begin( ), m_readData.end( ), m_cLeading ) : m_readData.begin( );
		auto msgEnd = m_bHasTerminating ? std::find( msgStart, m_readData.end( ), m_cTerminating ) : m_readData.end( );

		// While we found a message, process it
		while( msgEnd != m_readData.end( ) )
		{
			if( m_readCallback )
			{
				std::vector<char> messageData( msgStart, msgEnd );

				if( messageData.size( ) > 0 )
				{
					bool bIsCRCValid = true;

					if( m_bGenerateCRC )
					{
						uint8_t cCRC = 0;

						for( auto iter = msgStart; iter < msgEnd; iter++ )
						{
							cCRC += *iter;
						}

						if( !(cCRC == 0 || messageData[0] == '<') )
						{
							bIsCRCValid = false;
						}
					}

					if( bIsCRCValid )
					{
						// Return the whole message
						m_readCallback( messageData );
					}
					else
					{
						ROS_ERROR_STREAM_NAMED( "SerialIO", "Invalid CRC: " << ToHex( messageData ) );
					}
				}
				else
				{
					ROS_ERROR_STREAM_NAMED( "SerialIO", "Empty Message" );
				}
			}
			else
			{
				ROS_ERROR_NAMED( "StarGazerSerialIO", "Serial port data read but no callback specified!\n" );
			}

			// Remove the consumed message
			m_readData.erase( m_readData.begin( ), msgEnd );

			msgStart = m_bHasLeading ? std::find( m_readData.begin( ), m_readData.end( ), m_cLeading ) : m_readData.begin( );
			msgEnd = m_bHasTerminating ? std::find( msgStart, m_readData.end( ), m_cTerminating ) : m_readData.end( );
		}
	}
	else
	{
		ROS_ERROR_NAMED( "SerialIO", "Read Error: %s\n", error.message( ).c_str( ) );

		if( error == boost::asio::error::eof )
		{
			Close( );
		}
	}

	if( IsOpen( ) )
	{
		StartAsyncRead( );
	}
}

void SerialIO::OnCheckSerialPort( bool bInitialCheck, const boost::system::error_code& e )
{
	// Save current state (so we don't spam debug log on reconnect)
	bool bIsSerialOpen = m_bIsSerialOpen;

	m_bIsSerialOpen = IsOpen( );

	std::string strError;

	if( !m_bIsSerialOpen )
	{
		try
		{
			// Anonymous function call the message processor in the main ros thread
			m_SerialPort.open( m_strSerialPort );

			// Setup serial port for 8/N/1 operation @ 115.2kHz
			m_SerialPort.set_option( boost::asio::serial_port::baud_rate( 115200 ) );
			m_SerialPort.set_option( boost::asio::serial_port::flow_control( boost::asio::serial_port::flow_control::none ) );
			m_SerialPort.set_option( boost::asio::serial_port::character_size( 8 ) );
			m_SerialPort.set_option( boost::asio::serial_port::parity( boost::asio::serial_port::parity::none ) );
			m_SerialPort.set_option( boost::asio::serial_port::stop_bits( boost::asio::serial_port::stop_bits::one ) );

			StartAsyncRead( );

			m_bIsSerialOpen = true;
		}
	    catch( const std::exception& e )
	    {
	    	strError = e.what( );
	    }
	    catch( ... )
	    {
	    	strError = "Unknown error";
	    }

		if( !m_bIsSerialOpen && bInitialCheck )
		{
			ROS_ERROR( "Error connecting to serial port: %s (Retry: %.2f)\n", strError.c_str( ), m_fRetryTimeout );
		}
	}

	if( bIsSerialOpen != m_bIsSerialOpen )
	{
		if( m_bIsSerialOpen == true )
		{
			ROS_DEBUG( "Connected to serial port\n" );
		}
		else
		{
			ROS_ERROR( "Disconnected from serial port: %s (Retry: %.2f)\n", strError.c_str( ), m_fRetryTimeout );
		}
	}

	// If we failed to open the serial port keep trying
	if( !m_bIsSerialOpen )
	{
		m_oTimer->async_wait( std::bind( &SerialIO::OnCheckSerialPort, this, false, std::placeholders::_1 ) );
	}
}

} /* namespace srs */
