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
public:

	SerialIO						m_serial1;

	SerialIO						m_serial2;

	std::queue<std::vector<char>>	m_readData1;

	std::queue<std::vector<char>>	m_readData2;

	std::mutex              		m_mutex1;

	std::mutex              		m_mutex2;

	std::condition_variable			m_condition1;

	std::condition_variable			m_condition2;

public:
	SerialIOTest( )
	{

	}

	void OpenSerialPort( SerialIO& serial, std::string strPort,
		void (SerialIOTest::* callback) (std::vector<char>) )
	{
		serial.Open( strPort.c_str( ), std::bind( callback, this,
			std::placeholders::_1 ) );

		serial.EnableCRC( true );
		serial.SetIncludeLength( false );
		serial.SetTerminatingCharacter( '\n' );
		serial.SetEscapeCharacter( '\\' );
		serial.SetEscapeCharacters( std::set<char>( { '\\', '\n' } ) );

		EXPECT_TRUE( serial.IsOpen( ) );
	}

	void OpenSerialPort1( )
	{
		OpenSerialPort( m_serial1, g_strPort1, &SerialIOTest::ReadMessageFrom2 );
	}

	void OpenSerialPort2( )
	{
		OpenSerialPort( m_serial2, g_strPort2, &SerialIOTest::ReadMessageFrom1 );
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
	OpenSerialPort1( );

	OpenSerialPort2( );

	EXPECT_TRUE( m_serial1.IsOpen( ) );

	EXPECT_TRUE( m_serial2.IsOpen( ) );
}

TEST_F( SerialIOTest, TestClose )
{
	OpenSerialPort1( );

	OpenSerialPort2( );

	EXPECT_TRUE( m_serial1.IsOpen( ) );

	EXPECT_TRUE( m_serial2.IsOpen( ) );

	m_serial1.Close( );

	m_serial2.Close( );

	EXPECT_TRUE( !m_serial1.IsOpen( ) );

	EXPECT_TRUE( !m_serial2.IsOpen( ) );
}

TEST_F( SerialIOTest, TestSimpleReadWrite )
{
	OpenSerialPort1( );

	OpenSerialPort2( );

	std::string string1( "Hello" );
	std::vector<char> data1( string1.begin( ), string1.end( ) );

	m_serial1.Write( data1 );

	std::unique_lock<std::mutex> lock( m_mutex1 );

	m_condition1.wait( lock );

	std::vector<char> expectedData( data1.begin( ), data1.end( ) );

	// Add the CRC
//	expectedData.push_back( 12 );

	EXPECT_EQ( m_readData2.front( ), expectedData );
}

TEST_F( SerialIOTest, TestEscapeSequence )
{
	OpenSerialPort1( );

	OpenSerialPort2( );

	std::string string1( "\\AA\nBB\"" );
	std::vector<char> data1( string1.begin( ), string1.end( ) );

	m_serial1.Write( data1 );

	std::unique_lock<std::mutex> lock( m_mutex1 );

	m_condition1.wait( lock );

	std::vector<char> expectedData( data1.begin( ), data1.end( ) );
//
//	// Add the CRC
//	expectedData.push_back( 114 );

	EXPECT_EQ( m_readData2.front( ), expectedData );
}

TEST_F( SerialIOTest, TestMultipleReadWrites )
{
	OpenSerialPort1( );

	OpenSerialPort2( );

	m_serial2.EnableCRC( false );

	std::string string1( "Hello" );
	std::vector<char> data1( string1.begin( ), string1.end( ) );

	m_serial1.WriteRaw( std::vector<char>( ) );

	m_serial1.WriteRaw( data1 );
	usleep( 5000 );
	m_serial1.WriteRaw( data1 );
	usleep( 10000 );
	m_serial1.WriteRaw( data1 );
	usleep( 15000 );
	m_serial1.WriteRaw( std::vector<char>( { '\n' } ) );

	std::unique_lock<std::mutex> lock( m_mutex1 );

	m_condition1.wait( lock );

	std::string strExpected( "HelloHelloHello" );
	std::vector<char> expectedData( strExpected.begin( ), strExpected.end( ) );

	EXPECT_EQ( m_readData2.front( ), expectedData );
}

TEST_F( SerialIOTest, TestMultipleMessages )
{
	OpenSerialPort1( );

	OpenSerialPort2( );

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
	OpenSerialPort1( );

	OpenSerialPort2( );

	std::string dataLarge;

	// 65k of data
	for( int i : boost::irange( 0, 1024 ) )
	{
		// Add in increments of 64 bytes
		dataLarge += "1234567890123456789012345678901234567890123456789012345678901234";
	}

	std::vector<char> data1( dataLarge.begin( ), dataLarge.end( ) );

	try
	{
		m_serial1.Write( data1 );

		m_serial2.Write( data1 );

		// Closing while reading and writing is perfectly fine
		m_serial1.Close( );

		m_serial2.Close( );
    }
    catch( boost::system::system_error const & err )
    {
        FAIL( ) << "Expected std::exception";
    }
    catch( ... )
    {
        FAIL( ) << "Expected std::exception";
    }
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
