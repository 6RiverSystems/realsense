/*
 * SerialIO_test.cpp
 *
 *  Created on: Apr 4, 2016
 *      Author: dan
 */

#include <srslib_framework/io/SerialIO.hpp>
#include "gtest/gtest.h"
#include <cstdlib>
#include <unistd.h>
#include <queue>
#include <condition_variable>
#include <boost/range/irange.hpp>

using namespace srs;

namespace {

const std::string g_strPort1 = "/tmp/pty1";
const std::string g_strPort2 = "/tmp/pty2";

typedef std::vector<char> IOBuffer;

class SocatRunner
{
private:
	bool	m_bIsRunning;

public:

	SocatRunner( ) :
		m_bIsRunning( false )
	{
		Stop( );

		RemovePseudoTerminals( );

		usleep( 1000000 );
	}

	~SocatRunner( )
	{
		Stop( );

		RemovePseudoTerminals( );
	}

	void Start( )
	{
		if( !m_bIsRunning )
		{
			// Open up two linked pseudo serial ports
			std::string strStartCommand;
			strStartCommand += "touch " + g_strPort1 + " &&";
			strStartCommand += "touch " + g_strPort2 + " &&";
			strStartCommand += "socat -d -d pty,link=" + g_strPort1 + ",raw,echo=0 pty,link=" + g_strPort2 + ",raw,echo=0";
			strStartCommand += " &"; // Don't wait (background)

			// Startup socat
			printf ( "Starting socat: %s\n", strStartCommand.c_str( ) );
			std::system( strStartCommand.c_str( ) );

			m_bIsRunning = true;
		}
	}

	void Stop( )
	{
		if( m_bIsRunning )
		{
			// Kill socat
			std::string strStopCommand( "killall -1 socat" );

			printf( "Stopping socat: \"%s\"\n", strStopCommand.c_str( ) );
			std::system( strStopCommand.c_str( ) );

			m_bIsRunning = false;
		}
	}

	void RemovePseudoTerminals( )
	{
		std::string strRemoveTempTerminal( "rm " );
		strRemoveTempTerminal += g_strPort1;
		strRemoveTempTerminal += " ";
		strRemoveTempTerminal += g_strPort2;

		std::system( strRemoveTempTerminal.c_str( ) );
	}

} g_socat;

class SerialIOTest : public ::testing::Test
{
	enum class CONFIG
	{
		RAW,
		BRAIN_STEM,
		STAR_GAZER
	};

	enum class DATA
	{
		BASIC,
		CRC
	};

public:

	SerialIO						m_serial1;

	SerialIO						m_serial2;

	std::queue<std::vector<char>>	m_readData1;

	std::queue<std::vector<char>>	m_readData2;

	std::mutex              		m_mutex1;

	std::mutex              		m_mutex2;

	std::condition_variable			m_condition1;

	std::condition_variable			m_condition2;

	std::map<DATA, IOBuffer>		m_TestData;

public:
	SerialIOTest( )
	{
		m_TestData[DATA::BASIC] = { 'H', 'e', 'l', 'l', 'o' };
		m_TestData[DATA::BASIC] = { '6', 'R', 'i', 'v', 'e', 'r', 's' };
		m_TestData[DATA::BASIC] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F };
		m_TestData[DATA::BASIC] = { 'A', 'B', 'C', 'D', 'E', 'F', 'G', '\n' };
		m_TestData[DATA::BASIC] = { 'A', 'B', 'C', 'D', 'E', 'F', 'G', '\r' };
		m_TestData[DATA::CRC] = { 0x01, 0x02, 0x03, 0x04 };  // Special case where CRC == '\n'
		m_TestData[DATA::CRC] = { 0x10, 0x10, 0x10, 0x10, 0x10, 0x0C };  // Special case where CRC == '\\'
	}

	void OpenSerialPort( SerialIO& serial, std::string strPort, CONFIG eConfig,
		void (SerialIOTest::* callback) (std::vector<char>) )
	{
		serial.Open( strPort.c_str( ), std::bind( callback, this,
			std::placeholders::_1 ) );

		switch( eConfig )
		{
			case CONFIG::RAW:
			{
				// Nothing
			}
			break;

			case CONFIG::BRAIN_STEM:
			{
				serial.EnableCRC( true );
				serial.SetTerminatingCharacter( '\n' );
				serial.SetEscapeCharacter( '\\' );
			}
			break;

			case CONFIG::STAR_GAZER:
			{
				serial.SetLeadingCharacter( '~' );
				serial.SetTerminatingCharacter( '`' );
				serial.SetFirstByteDelay( std::chrono::microseconds( 30000 ) );
				serial.SetByteDelay( std::chrono::microseconds( 5000 ) );
			}
			break;
		}

		EXPECT_TRUE( serial.IsOpen( ) );
	}

	void OpenSerialPort1( CONFIG eConfig = CONFIG::BRAIN_STEM )
	{
		OpenSerialPort( m_serial1, g_strPort1, eConfig, &SerialIOTest::ReadMessageFrom2 );
	}

	void OpenSerialPort2( CONFIG eConfig = CONFIG::BRAIN_STEM )
	{
		OpenSerialPort( m_serial2, g_strPort2, eConfig, &SerialIOTest::ReadMessageFrom1 );
	}

	void OpenSerialPorts( CONFIG eConfig = CONFIG::BRAIN_STEM )
	{
		OpenSerialPort1( );

		OpenSerialPort2( );
	}

	void CloseSerialPort( SerialIO& serial )
	{
		serial.Close( );

		EXPECT_TRUE( !serial.IsOpen( ) );
	}

	void CloseSerialPorts( )
	{
		CloseSerialPort( m_serial1 );
		CloseSerialPort( m_serial2 );
	}

	void SetUp( )
	{

	}

	void TearDown( )
	{
		std::queue<std::vector<char>> emptyQueue;

		m_readData1.swap( emptyQueue );

		m_readData2.swap( emptyQueue );

		m_serial1.Close( );

		m_serial2.Close( );
	}

	bool CompareBuffers( const std::vector<char>& buffer1, const std::vector<char>& buffer2 )
	{
		return buffer1.size( ) == buffer2.size( ) &&
			memcmp( buffer1.data( ), buffer2.data( ), buffer1.size( ) );
	}

	void ReadMessageFrom1( std::vector<char> buffer )
	{
		m_readData2.push( buffer );

		std::unique_lock<std::mutex> lock( m_mutex1 );

		m_condition1.notify_one( );
	}

	void ReadMessageFrom2( std::vector<char> buffer )
	{
		m_readData1.push( buffer );

		std::unique_lock<std::mutex> lock( m_mutex2 );

		m_condition2.notify_one( );
	}

	void WaitForData( size_t numberOfMessages )
	{
		// Wait for for messages to come in or timeout
		while( true )
		{
			std::unique_lock<std::mutex> lock( m_mutex1 );

			if( m_condition1.wait_until( lock, std::chrono::steady_clock::now( ) + std::chrono::milliseconds( 500 ) ) == std::cv_status::timeout )
			{
				break;
			}
			else
			{
				if( m_readData2.size( ) == numberOfMessages )
				{
					break;
				}
			}
		}
	}

	~SerialIOTest( )
	{

	}

   // put in any custom data members that you need
};

TEST_F( SerialIOTest, OpenInvalidSerialPort )
{
	m_serial1.Open( "/foobar", std::bind( &SerialIOTest::ReadMessageFrom2, this,
		std::placeholders::_1 ) );

	EXPECT_FALSE( m_serial1.IsOpen( ) );
}

TEST_F( SerialIOTest, TestSpinUntilOpen )
{
	// Try once when it is closed, then spin until it connects (wait for 3 seconds)
	for( int i : boost::irange( 0, 300 ) )
	{
		m_serial1.Open( g_strPort1.c_str( ), std::bind( &SerialIOTest::ReadMessageFrom2, this,
			std::placeholders::_1 ) );

		// If we are able to open when socat is not running, we have failed
		if( i == 0 )
		{
	    	ASSERT_FALSE( m_serial1.IsOpen( ) );
		}

		// Start it after the first failure
		if( i == 0 )
		{
			g_socat.Start( );
		}
		else
		{
			if( m_serial1.IsOpen( ) )
			{
				break;
			}
		}

		// Wait for 10ms
		usleep( 100000 );
	}

	EXPECT_TRUE( m_serial1.IsOpen( ) );
}

TEST_F( SerialIOTest, TestOpen )
{
	OpenSerialPorts( );
}

TEST_F( SerialIOTest, TestClose )
{
	OpenSerialPorts( );

	CloseSerialPorts( );
}

TEST_F( SerialIOTest, TestSimpleReadWrite )
{
	OpenSerialPorts( );

	for( auto data : m_TestData )
	{
		m_serial1.Write( data.second );
	}

	WaitForData( m_TestData.size( ) );

	for( auto data : m_TestData )
	{
		EXPECT_EQ( m_readData2.front( ), data.second );

		m_readData2.pop( );
	}
}

TEST_F( SerialIOTest, TestBasicEscape )
{
	OpenSerialPorts( );

	std::string string1( "\\AA\nBB\"" );
	std::vector<char> data1( string1.begin( ), string1.end( ) );

	m_serial1.Write( data1 );

	WaitForData( 1 );

	EXPECT_EQ( m_readData2.front( ), data1 );
}

TEST_F( SerialIOTest, TestEscapeInCRC )
{
	OpenSerialPorts( );

	std::vector<char> data( { '\\', 'A', 'A', '\n', 'B', 'B', '"' } );

	m_serial1.Write( data );

	WaitForData( 1 );

	EXPECT_EQ( m_readData2.front( ), data );
}

TEST_F( SerialIOTest, TestEscapeCarryOver )
{
	OpenSerialPorts( );

	std::string string1( "\\AA\nBB\"" );
	std::vector<char> data1( string1.begin( ), string1.end( ) );

	m_serial1.Write( data1 );

	std::unique_lock<std::mutex> lock( m_mutex1 );

	m_condition1.wait_until( lock, std::chrono::steady_clock::now( ) + std::chrono::milliseconds( 500 ) );

	EXPECT_EQ( m_readData2.front( ), data1 );
}

TEST_F( SerialIOTest, TestEscapeReset )
{
	OpenSerialPorts( );

	std::string string1( "\\AA\nBB\"" );
	std::vector<char> data1( string1.begin( ), string1.end( ) );

	m_serial1.Write( data1 );

	std::unique_lock<std::mutex> lock( m_mutex1 );

	m_condition1.wait_until( lock, std::chrono::steady_clock::now( ) + std::chrono::milliseconds( 500 ) );

	EXPECT_EQ( m_readData2.front( ), data1 );
}

TEST_F( SerialIOTest, TestValidCRC )
{

}

TEST_F( SerialIOTest, TestInvalidCRC )
{

}

TEST_F( SerialIOTest, TestStartInMiddleOfMessage )
{
	OpenSerialPorts( );

	std::string string1( "\\AA\nBB\"" );
	std::vector<char> data1( string1.begin( ), string1.end( ) );

	m_serial1.Write( data1 );

	std::unique_lock<std::mutex> lock( m_mutex1 );

	m_condition1.wait_until( lock, std::chrono::steady_clock::now( ) + std::chrono::milliseconds( 500 ) );

	EXPECT_EQ( m_readData2.front( ), data1 );
}

TEST_F( SerialIOTest, TestLeadingAndTrailing )
{

}

TEST_F( SerialIOTest, TestMultipleReadWrites )
{
	OpenSerialPorts( );

	m_serial1.EnableCRC( false );
	m_serial2.EnableCRC( false );

	std::string string1( "Hello" );
	std::vector<char> data1( string1.begin( ), string1.end( ) );

	m_serial1.Write( data1 );
	usleep( 5000 );
	m_serial1.Write( data1 );
	usleep( 10000 );
	m_serial1.Write( data1 );
	usleep( 15000 );

	for( int i : boost::irange( 0, 3 ) )
	{
		std::unique_lock<std::mutex> lock( m_mutex1 );

		// Wait for next message if necessary
		if( m_readData2.size( ) == 0 )
		{
			m_condition1.wait( lock );
		}

		EXPECT_EQ( m_readData2.front( ), data1 );

		m_readData2.pop( );
	}
}

TEST_F( SerialIOTest, TestMultipleMessages )
{
	OpenSerialPorts( );

	std::string string1( "HelloHelloHelloHelloHelloHelloHelloHelloHello" );
	std::vector<char> data1( string1.begin( ), string1.end( ) );

	m_serial1.Write( data1 );

	usleep( 10000 );

	m_serial1.Write( data1 );

	usleep( 500 );

	m_serial1.Write( data1 );

	usleep( 10000 );

	m_serial1.Write( data1 );

	usleep( 20000 );

	m_serial1.Write( data1 );

	for( int i : boost::irange( 0, 5 ) )
	{
		std::unique_lock<std::mutex> lock( m_mutex1 );

		// Wait for next message if necessary
		if( m_readData2.size( ) == 0 )
		{
			m_condition1.wait( lock );
		}

		EXPECT_EQ( std::string( m_readData2.front( ).begin( ), m_readData2.front( ).end( ) ), string1 );

		m_readData2.pop( );
	}
}

TEST_F( SerialIOTest, TestCloseWhileReadWrite )
{
	OpenSerialPorts( );

	std::string dataLarge;

	// 65k of data
	for( int i : boost::irange( 0, 1024 ) )
	{
		// Add in increments of 64 bytes
		dataLarge += "1234567890123456789012345678901234567890123456789012345678901234";
	}

	std::vector<char> data1( dataLarge.begin( ), dataLarge.end( ) );

	m_serial1.Write( data1 );

	m_serial2.Write( data1 );

	// Closing while reading and writing is perfectly fine
	m_serial1.Close( );

	m_serial2.Close( );
}

TEST_F( SerialIOTest, TestBandwidth )
{
	// Test that our data rate is in the ballpark of what it should be (tolerance)
}

}  // namespace

int main(int argc, char **argv)
{
	g_socat.Stop( );

	::testing::InitGoogleTest( &argc, argv );

	int success = RUN_ALL_TESTS( );

	g_socat.Stop( );

	return success;
}
