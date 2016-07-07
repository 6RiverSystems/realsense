/*
 * Test_MessageProcessor.cpp
 *
 *  Created on: Apr 4, 2016
 *      Author: dan
 */
#include "gtest/gtest.h"
#include <srslib_framework/MsgOperationalState.h>

namespace srs {

class MessageProcessor : public ::testing::Test
{
public:

public:

	MessageProcessor( )
	{

	}

	void SetUp( )
	{

	}

	void TearDown( )
	{

	}

	~MessageProcessor( )
	{

	}

};

TEST_F( MessageProcessor, TestProtocol )
{

}

}  // namespace

int main(int argc, char **argv)
{
	::testing::InitGoogleTest( &argc, argv );

	return RUN_ALL_TESTS( );
}
