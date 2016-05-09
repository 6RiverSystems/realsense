/*
 * SerialIO.cpp
 *
 *  Created on: Apr 4, 2016
 *      Author: dan
 */

#include <srslib_framework/io/SerialIO.hpp>
#include <srslib_framework/utils/Logging.hpp>
#include <srslib_framework/utils/Thread.hpp>
#include <thread>
#include <ros/ros.h>

namespace srs {

SerialIO::SerialIO( ) :
	m_Thread( ),
	m_oWork( m_IOService ),
	m_SerialPort( m_IOService ),
	m_strSerialPort( ),
	m_bIsSerialOpen( false ),
	m_fRetryTimeout( 1.0 ),
	m_bIsWriting( false ),
	m_readBuffer( 1024 ),
	m_writeBuffer( ),
	m_readState( READ_STATE::DEFAULT ),
	m_readPartialData( ),
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
	m_strSerialPort = pszName;

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

	m_strSerialPort = "";
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
	m_bGenerateCRC = bGenerateCRC;
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
	uint32_t dwStartIndex = m_writeBuffer.size( );

	if( !bIsRaw )
	{
		// Add length
		if( m_bIncludeLength )
		{
			m_writeBuffer.push_back( (uint8_t)buffer.size( ) );
		}
	}

	// Add payload data
	m_writeBuffer.insert( m_writeBuffer.end( ), buffer.begin( ), buffer.end( ) );

	if( !bIsRaw && ( m_bGenerateCRC || m_setCharsToEscape.size( ) ) )
	{
		uint8_t cCRC = 0;

		// Escape characters and generate CRC
		for( std::size_t dwIndex = dwStartIndex; dwIndex != m_writeBuffer.size( ); dwIndex++ )
		{
			cCRC += m_writeBuffer[dwIndex];

			// Escape the charactem_writeBufferr
			if( m_setCharsToEscape.find( m_writeBuffer[dwIndex] ) != m_setCharsToEscape.end( ) )
			{
				m_writeBuffer.insert( m_writeBuffer.begin( ) + dwIndex, m_cEscape );

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
				m_writeBuffer.push_back( m_cEscape );
			}

			m_writeBuffer.push_back( -cCRC );
		}

		// Add terminating character
		if( m_cTerminating )
		{
			m_writeBuffer.push_back( m_cTerminating );
		}
	}

	// If we are already trying to write, then wait until we recieve the callback
	if( !m_bIsWriting )
	{
		m_bIsWriting = true;

		m_interByteDelay = m_firstByteDelay;

		// If we have a first byte delay only write one byte
		size_t writeSize = (m_interByteDelay == std::chrono::microseconds( )) ? m_writeBuffer.size( ) : 1 ;

		m_SerialPort.async_write_some( boost::asio::buffer( m_writeBuffer.data( ), writeSize ),
		    std::bind( &SerialIO::OnWriteComplete, this, std::placeholders::_1, std::placeholders::_2 ) );
	}
}

void SerialIO::StartAsyncRead( )
{
	m_SerialPort.async_read_some( boost::asio::buffer( m_readBuffer, m_readBuffer.size( ) ),
    	std::bind( &SerialIO::OnReadComplete, this, std::placeholders::_1, std::placeholders::_2 ) );
}

void SerialIO::OnWriteComplete( const boost::system::error_code& error, std::size_t size )
{
//	ROS_DEBUG_STREAM_NAMED( "SerialIO", "Write Complete: " <<
//		ToHex( std::vector<char>( m_writeBuffer.begin( ), m_writeBuffer.begin( ) + size ) ) );

	std::vector<char>( m_writeBuffer.begin( ) + size, m_writeBuffer.end( ) ).swap( m_writeBuffer );

	if( m_writeBuffer.begin( ) != m_writeBuffer.end( ) )
	{
		std::this_thread::sleep_for( m_interByteDelay );

		m_interByteDelay = m_byteDelay;

		size_t writeSize = (m_interByteDelay == std::chrono::microseconds( )) ? m_writeBuffer.size( ) : 1 ;

		m_SerialPort.async_write_some( boost::asio::buffer( m_writeBuffer.data( ), writeSize ),
			std::bind( &SerialIO::OnWriteComplete, this, std::placeholders::_1, std::placeholders::_2 ) );
	}
	else
	{
		m_bIsWriting = false;
	}
}

void SerialIO::OnReadComplete( const boost::system::error_code& error, std::size_t size )
{
	if( !error )
	{
		uint8_t cCRC = 0;

		// Start with the partially parsed message
		std::vector<char> messageData( m_readPartialData.begin( ), m_readPartialData.end( ) );

//		ROS_DEBUG_STREAM_NAMED( "SerialIO", "Left over read data: " <<
//			ToHex( std::vector<char>( messageData.begin( ), messageData.end( ) ) ) );

		// If we do not have a leading character, then we are always in a message
		if( m_readState == READ_STATE::DEFAULT &&
			!m_bHasLeading )
		{
			m_readState = READ_STATE::IN_MESSAGE;
		}

		for( auto iter = m_readBuffer.begin( ); iter < m_readBuffer.begin( ) + size; iter++ )
		{
//			ROS_ERROR_NAMED( "SerialIO", "Char: %c", *iter );

			if( m_bHasEscape &&
				*iter == m_cEscape )
			{
				if( m_readState == READ_STATE::IN_MESSAGE_ESCAPED )
				{
					messageData.push_back( *iter );

					cCRC += *iter;

					m_readState = READ_STATE::IN_MESSAGE;
				}
				else
				{
					m_readState = READ_STATE::IN_MESSAGE_ESCAPED;
				}
			}
			else if( m_readState == READ_STATE::DEFAULT )
			{
				// Should never get into this case if we are not a leading character
				assert( !m_bHasLeading );

				// Only start a message when we encounter a leading character
				if( *iter == m_cLeading )
				{
					m_readState = READ_STATE::IN_MESSAGE;
				}
				else
				{
					ROS_ERROR_STREAM_NAMED( "SerialIO", "Received partial message, waiting for leading character" );
				}
			}
			else if( m_readState == READ_STATE::IN_MESSAGE_ESCAPED ||
				*iter != m_cTerminating )
			{
				messageData.push_back( *iter );

				cCRC += *iter;

				m_readState = READ_STATE::IN_MESSAGE;
			}
			else
			{
				if( messageData.size( ) > 0 )
				{
					if( messageData[0] == '<' )
					{
						cCRC = 0;
					}
					else if( m_bGenerateCRC )
					{
						messageData.pop_back( );
					}

					if( cCRC == 0 )
					{
						if( m_readCallback )
						{
							m_readCallback( messageData );
						}
						else
						{
							ROS_ERROR_NAMED( "SerialIO", "Serial port data read but no callback specified!\n" );
						}
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

				messageData.clear( );

				cCRC = 0;
			}
		}

//		ROS_DEBUG_STREAM_NAMED( "SerialIO", "Left over read data: " <<
//			ToHex( std::vector<char>( m_readPartialData.begin( ), m_readPartialData.end( ) ) ) );

		// Left over partial message
		m_readPartialData = messageData;
	}
	else
	{
		if( IsOpen( ) )
		{
			ROS_ERROR_NAMED( "SerialIO", "Read Error: %s", error.message( ).c_str( ) );

			if( error == boost::asio::error::eof )
			{
				Close( );
			}
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

			m_readState = READ_STATE::DEFAULT;

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
			ROS_DEBUG( "Error connecting to serial port (%s): %s (Retry: %.2f)", m_strSerialPort.c_str( ),
				strError.c_str( ), m_fRetryTimeout );
		}
	}

	if( bIsSerialOpen != m_bIsSerialOpen )
	{
		if( m_bIsSerialOpen == true )
		{
			ROS_INFO( "Connected to serial port" );
		}
		else
		{
			ROS_ERROR( "Disconnected from serial port: %s (Retry: %.2f)", strError.c_str( ), m_fRetryTimeout );
		}
	}

	// If we failed to open the serial port keep trying
	if( !m_bIsSerialOpen )
	{
		m_oTimer->async_wait( std::bind( &SerialIO::OnCheckSerialPort, this, false, std::placeholders::_1 ) );
	}
}

} /* namespace srs */
