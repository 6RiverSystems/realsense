/*
 * SerialIO_test.cpp
 *
 *  Created on: Apr 4, 2016
 *      Author: dan
 */

#include "SerialIO.h"
#include "gtest/gtest.h"
#include <cstdlib>

using namespace srs;

namespace {

class SerialIOTest : public ::testing::Test
{
private:

public:
	SerialIOTest( )
	{

	}

	void SetUp( )
	{

	}

	void TearDown( )
	{

	}

	void ReadCallback( std::vector<char> buffer )
	{

	}

	~SerialIOTest( )
	{

	}

   // put in any custom data members that you need
};

TEST_F( SerialIOTest, OpenInvalidSerialPort )
{
    try
    {
    	SerialIO serial;
    	serial.Open( "/foobar", std::bind( &SerialIOTest::ReadCallback, this,
    		std::placeholders::_1 ) );

    	FAIL( ) << "Expected std::exception";
    }
    catch( boost::system::system_error const & err )
    {
    	const char* pszError = err.what( );

    	ASSERT_STREQ( "open: No such file or directory", pszError );
    }
    catch( ... )
    {
        FAIL( ) << "Expected std::exception";
    }
}

TEST_F( SerialIOTest, OpenSuccess )
{
    try
    {
    	SerialIO serial;
    	serial.Open( "/tmp/pty1", std::bind( &SerialIOTest::ReadCallback, this,
    		std::placeholders::_1 ) );

    	EXPECT_TRUE( serial.IsOpen( ) );
    }
    catch( const std::exception& e )
    {
        FAIL( ) << "Open failed: " <<  e.what( );
    }
    catch( ... )
    {
        FAIL( ) << "Open failed: unknown exception";
    }
}

TEST_F( SerialIOTest, Close )
{
    try
    {
    	SerialIO serial;
    	serial.Open( "/tmp/pty1", std::bind( &SerialIOTest::ReadCallback, this,
    		std::placeholders::_1 ) );

    	EXPECT_TRUE( serial.IsOpen( ) );

    	serial.Close( );

    	EXPECT_FALSE( serial.IsOpen( ) );
    }
    catch( const std::exception& e )
    {
        FAIL( ) << "Open failed: " <<  e.what( );
    }
    catch( ... )
    {
        FAIL( ) << "Open failed: unknown exception";
    }}

}  // namespace

int main(int argc, char **argv) {

	// Open up two pseudo serial ports that communicate with each other
	// /tmp/pty1 and /tmp/pty2
	std::string strCommand;
	strCommand += "touch /tmp/pty1 &&";
	strCommand += "touch /tmp/pty2 &&";
	strCommand += "socat -d -d pty,link=/tmp/pty1,raw,echo=0 pty,link=/tmp/pty2,raw,echo=0";
	strCommand += " &"; // Don't wait (background)

	std::system( strCommand.c_str( ) );

	sleep( 1 );

	::testing::InitGoogleTest(&argc, argv);

	int success = RUN_ALL_TESTS( );

	std::system( "killall socat" );

	return success;

}
