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

namespace srs
{

SerialIO::SerialIO( const char* pszName ) :
	m_strName( pszName ),
	m_strDebug( "serial-io." + m_strName ),
	m_Thread( ),
	m_serialThreadId( ),
	m_oWork( m_IOService ),
	m_SerialPort( m_IOService ),
	m_strSerialPort( ),
	m_bIsSerialOpen( false ),
	m_fRetryTimeout( 1.0 ),
	m_bIsWriting( false ),
	m_readBuffer( 1024 ),
	m_writeBuffer( ),
	message_( ),
	m_readState( READ_STATE::DEFAULT ),
	m_cCRC( 0 ),
	m_readCallback( ),
	m_bEnableCRC( false ),
	m_bHasLeading( false ),
	m_cLeading( 0 ),
	m_bHasTerminating( false ),
	m_cTerminating( 0 ),
	m_bHasEscape( false ),
	m_cEscape( 0 ),
	m_firstByteDelay( ),
	m_byteDelay( ),
	m_interByteDelay( )
#if defined( ENABLE_TEST_FIXTURE )
	,
	m_highrezclk( ),
	m_lastTime( std::chrono::milliseconds( 0 ) ),
	m_messageTiming( ),
	m_vecMessageTiming( )
#endif
{
	m_oTimer.reset( new boost::asio::deadline_timer( m_IOService ) );

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

void SerialIO::Open( const char* pszName, ConnectionCallbackFn connectionCallback,
	ReadCallbackFn readCallback )
{
	if( !IsOpen( ) )
	{
		std::condition_variable condition;

		std::mutex mutex;

		m_IOService.post( [&]( )
			{
				m_strSerialPort = pszName;

				m_connectionCallback = connectionCallback;

				m_readCallback = readCallback;

				m_serialThreadId = std::this_thread::get_id( );

				OnCheckSerialPort( false );

				condition.notify_one( );
			} );


		std::unique_lock<std::mutex> lock( mutex );

		condition.wait( lock );
	}
	else
	{
		ROS_ERROR_NAMED( m_strDebug, "%s: Attempt to open serial port but it is already open.",
			m_strName.c_str( ) );
	}
}

bool SerialIO::IsOpen( ) const
{
	return m_SerialPort.is_open( );
}

void SerialIO::Close( )
{
	if( m_SerialPort.is_open( ) )
	{
		m_SerialPort.close( );
	}
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
	m_bEnableCRC = bGenerateCRC;
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

void SerialIO::SetFirstByteDelay( std::chrono::microseconds firstByteDelay )
{
	m_firstByteDelay = firstByteDelay;
}

std::chrono::microseconds SerialIO::GetFirstByteDelay( )
{
	return m_firstByteDelay;
}

void SerialIO::SetByteDelay( std::chrono::microseconds byteDelay )
{
	m_byteDelay = byteDelay;
}

std::chrono::microseconds SerialIO::GetByteDelay( )
{
	return m_byteDelay;
}

void SerialIO::Write( const std::vector<char>& buffer )
{
//	ROS_DEBUG_STREAM_NAMED( m_strDebug, "%s: Write: " << m_strName.c_str( ), ToHex( buffer ) );

	if( IsOpen( ) )
	{
		// Do all data operations in serial processing thread
		m_IOService.post( std::bind( &SerialIO::WriteInSerialThread, this, buffer ) );
	}
	else
	{
		// Throw error
		throw std::runtime_error( "port not open" );
	}
}

void SerialIO::WriteInSerialThread( std::vector<char> writeBuffer )
{
//	ROS_DEBUG_STREAM_NAMED( m_strDebug, m_strName << ": WriteInSerialThread: " <<
//		ToHex( writeBuffer ) );

	assert( m_serialThreadId == std::this_thread::get_id( ) );

	// Add leading character
	if( m_bHasLeading )
	{
		m_writeBuffer.push_back( m_cLeading );
	}

	uint8_t cCRC = 0;

	// Function to add an escaped char (if necessary) to the array
	auto addEscapedCharacter =
		[&](const char& cChar )
		{
			if( m_bHasEscape )
			{
				if( ( cChar == m_cEscape )					 ||
					( m_bHasLeading && cChar == m_cLeading ) ||
					( m_bHasTerminating && cChar == m_cTerminating ) )
				{
					m_writeBuffer.push_back( m_cEscape );
				}
			}

			m_writeBuffer.push_back( cChar );
		};

	for( const char& cChar : writeBuffer )
	{
		cCRC += cChar;

		addEscapedCharacter( cChar );
	}

	// Add CRC
	if( m_bEnableCRC )
	{
		addEscapedCharacter( -cCRC );
	}

	// Add terminating character
	if( m_bHasTerminating )
	{
		m_writeBuffer.push_back( m_cTerminating );
	}

	// If we are already trying to write, then wait until we receive the callback
	if( !m_bIsWriting )
	{
		m_bIsWriting = true;

		m_interByteDelay = m_firstByteDelay;

		// If we have a first byte delay only write one byte
		size_t writeSize = m_interByteDelay.count( ) ?  1 : m_writeBuffer.size( );

		ROS_DEBUG_NAMED( m_strDebug, "%s: Serial Write: %s", m_strName.c_str( ),
			ToHex( m_writeBuffer ).c_str( ) );

		m_SerialPort.async_write_some( boost::asio::buffer( m_writeBuffer.data( ), writeSize ),
			std::bind( &SerialIO::OnWriteComplete, this, std::placeholders::_1, std::placeholders::_2 ) );
	}
}

void SerialIO::StartAsyncTimer( )
{
	m_oTimer->cancel( );

	m_oTimer->expires_from_now( boost::posix_time::seconds( m_fRetryTimeout ) );

	m_oTimer->async_wait( [&]( const boost::system::error_code& e )
		{
			OnCheckSerialPort( false, e );
		} );
}

void SerialIO::StartAsyncRead( )
{
	m_SerialPort.async_read_some( boost::asio::buffer( m_readBuffer, m_readBuffer.size( ) ),
		std::bind( &SerialIO::OnReadComplete, this, std::placeholders::_1, std::placeholders::_2 ) );
}

void SerialIO::OnWriteComplete( const boost::system::error_code& error, std::size_t size )
{
	assert( m_serialThreadId == std::this_thread::get_id( ) );

//	ROS_DEBUG_STREAM_NAMED( m_strDebug, m_strName << ": Write Complete (" << size << ", " << m_writeBuffer.size( ) << "):" <<
//		ToHex( std::vector<char>( m_writeBuffer.begin( ), m_writeBuffer.begin( ) + size ) ) );

	std::vector<char>( m_writeBuffer.begin( ) + size, m_writeBuffer.end( ) ).swap( m_writeBuffer );

//	ROS_DEBUG_STREAM_NAMED( m_strDebug, m_strName << ": Write Complete remaining (" << m_writeBuffer.size( ) << "):" <<
//		ToHex( m_writeBuffer ) );

	if( m_writeBuffer.begin( ) != m_writeBuffer.end( ) )
	{
//		ROS_DEBUG_STREAM_NAMED( m_strDebug, "Write Bytes: sleep for " << m_interByteDelay.count( ) );

		std::this_thread::sleep_for( m_interByteDelay );

		m_interByteDelay = m_byteDelay;

		size_t writeSize = m_interByteDelay.count( ) ? 1 : m_writeBuffer.size( );

		m_SerialPort.async_write_some( boost::asio::buffer( m_writeBuffer.data( ), writeSize ),
			std::bind( &SerialIO::OnWriteComplete, this, std::placeholders::_1, std::placeholders::_2 ) );

		// Reset the first byte delay if we encounter a start or end of message
		if( size == 1 && m_firstByteDelay.count( ) )
		{
			if( m_bHasLeading )
			{
				if( m_cLeading == m_writeBuffer[0] )
				{
					m_interByteDelay = m_firstByteDelay;
				}
			}
			else if( m_bHasTerminating )
			{
				if( m_cTerminating == m_writeBuffer[0] )
				{
					m_interByteDelay = m_firstByteDelay;
				}
			}
		}
	}
	else
	{
		m_bIsWriting = false;
	}
}

void SerialIO::OnReadComplete( const boost::system::error_code& error, std::size_t size )
{
	#if defined( ENABLE_TEST_FIXTURE )

		auto now = m_highrezclk.now( );

	#endif

	assert( m_serialThreadId == std::this_thread::get_id( ) );

	if( !error )
	{
		// Start with the last parsed message
		// Add room for the new data
		message_.reserve(message_.size() + size);

		size_t messageStart = 0;

		auto changeState =
			[&]( const READ_STATE& eNewState )
			{
				#if defined( ENABLE_TEST_FIXTURE )
					if( m_readState == READ_STATE::DEFAULT &&
						eNewState == READ_STATE::IN_MESSAGE )
					{
//						ROS_DEBUG_NAMED( m_strDebug, "Time reset" );

						m_lastTime = now;
					}
				#endif

				m_readState = eNewState;
			};

		// Function to add an escaped char (if necessary) to the array
		auto addCharacter =
			[&]( const char& cChar )
			{
				message_.push_back(cChar);

				m_cCRC += cChar;
			};

		for( int i = 0; i < size; i++ )
		{
			#if defined( ENABLE_TEST_FIXTURE )

				ROS_DEBUG_NAMED( m_strDebug, "Time between reads %d: %d ", messageData.size( ), std::chrono::duration_cast<std::chrono::microseconds>(now - m_lastTime).count( ) );

				m_messageTiming.push_back( std::chrono::duration_cast<std::chrono::microseconds>(now - m_lastTime) );

				m_lastTime = now;

			#endif

			// If we do not have a leading character, then we are always in a message
			if( m_readState == READ_STATE::DEFAULT && !m_bHasLeading )
			{
				changeState( READ_STATE::IN_MESSAGE );
			}

//			ROS_ERROR_NAMED( m_strDebug, "Char: %02x", (unsigned char)m_readBuffer[i] );

			if( m_bHasEscape &&
				m_readBuffer[i] == m_cEscape )
			{
				if( m_readState == READ_STATE::IN_MESSAGE_ESCAPED )
				{
					addCharacter( m_readBuffer[i] );

					changeState( READ_STATE::IN_MESSAGE );
				}
				else
				{
					changeState( READ_STATE::IN_MESSAGE_ESCAPED );
				}
			}
			else if( m_readState == READ_STATE::DEFAULT )
			{
				// Should never get into this case if we are not a leading character
				assert( m_bHasLeading );

				// Only start a message when we encounter a leading character
				if( m_readBuffer[i] == m_cLeading )
				{
					changeState( READ_STATE::IN_MESSAGE );
				}
			}
			else if( m_readState == READ_STATE::IN_MESSAGE_ESCAPED ||
					m_readBuffer[i] != m_cTerminating )
			{
				addCharacter( m_readBuffer[i] );

				changeState( READ_STATE::IN_MESSAGE );
			}
			else
			{
				if( message_.size() > 0 )
				{
					if( message_[0] == '<' )
					{
						m_cCRC = 0;
					}
					else if( m_bEnableCRC )
					{
						// Don't remove the crc if we failed
						if( m_cCRC == 0 )
						{
							message_.pop_back( );
						}
					}
					else
					{
						// We are not using a CRC
						m_cCRC = 0;
					}

					if( m_cCRC == 0 )
					{
						if( m_readCallback )
						{
//							ROS_DEBUG_STREAM_NAMED( m_strDebug, "ReadData (" << messageSize_ << "): " <<
//								ToHex( std::vector<char>( message_.begin( ), message_.begin( ) + messageSize_ ) ) );

							m_readCallback(message_);

							message_ = std::vector<char>();
						}
						else
						{
							ROS_ERROR_NAMED( m_strDebug, "Serial port data read but no callback specified!\n" );
						}
					}
					else
					{
						ROS_ERROR_STREAM_NAMED( m_strDebug, "Invalid CRC (" << message_.size() << "): " <<
							ToHex( std::vector<char>( message_.begin( ), message_.begin( ) + message_.size() ) ) <<
							"(CRC: " << ToHex( std::vector<char>( { (char)m_cCRC } ) ) << ")" );
					}
				}
				else
				{
					ROS_ERROR_STREAM_NAMED( m_strDebug, "Empty Message" );
				}

				m_cCRC = 0;

				changeState( READ_STATE::DEFAULT );

				#if defined( ENABLE_TEST_FIXTURE )

					m_vecMessageTiming.push_back( m_messageTiming );

					m_messageTiming.clear( );

				#endif
			}
		}
	}
	else
	{
		if( IsOpen( ) )
		{
			ROS_ERROR_NAMED( m_strDebug, "Read Error: %s", error.message( ).c_str( ) );

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
	else
	{
		StartAsyncTimer( );
	}
}

bool SerialIO::OnCheckSerialPort( bool bInitialCheck, const boost::system::error_code& e )
{
	assert( m_serialThreadId == std::this_thread::get_id( ) );

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

			m_cCRC = 0;

			#if defined( ENABLE_TEST_FIXTURE )

				m_vecMessageTiming.clear( );

			#endif

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
			ROS_DEBUG_NAMED( m_strDebug, "%s: Error connecting to serial port (%s): %s (Will retry every %.2f seconds)",
				m_strName.c_str( ), m_strSerialPort.c_str( ), strError.c_str( ), m_fRetryTimeout );
		}
	}

	if( bIsSerialOpen != m_bIsSerialOpen )
	{
		if( m_bIsSerialOpen == true )
		{
			ROS_INFO_NAMED( m_strDebug, "%s: Connected to serial port: %s",
				m_strName.c_str( ), m_strSerialPort.c_str( ) );
		}
		else
		{
			ROS_ERROR_NAMED( m_strDebug, "%s: Disconnected from serial port (%s): %s (Will retry every %.2f seconds)",
				m_strName.c_str( ), m_strSerialPort.c_str( ), strError.c_str( ), m_fRetryTimeout );
		}

		if( m_connectionCallback )
		{
			m_connectionCallback( m_bIsSerialOpen );
		}
	}

	// If we failed to open the serial port keep trying
	if( m_bIsSerialOpen )
	{
		StartAsyncRead( );
	}
	else
	{
		StartAsyncTimer( );
	}

	return m_bIsSerialOpen;
}

} /* namespace srs */
