/*
 * SerialIO_test.cpp
 *
 *  Created on: Apr 4, 2016
 *      Author: dan
 */

#include "SerialIO.h"

#include "gtest/gtest.h"

namespace {

class SerialIOTest : public ::testing::Test {};

TEST( SerialIOTest, Open )
{
	EXPECT_EQ(0, 0);
}

TEST( SerialIOTest, Close )
{
	EXPECT_EQ(0, 0);
}

}  // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
