/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#include <gtest/gtest.h>

#include <vector>
#include <srsdrv_brainstem/BrainStemMessages.h>

using namespace std;
using namespace srs;

#include <srslib_test/utils/Compare.hpp>

#include <hw_message/RawOdometryHandler.hpp>

using namespace srslib_framework;

vector<OdometryRPM>& getRawOdometryTestMsgs()
{
	static vector<OdometryRPM> testMessages;

	if (!testMessages.size())
	{
		OdometryRPM rawOdometry;
		rawOdometry.left_wheel_rpm = 1.0f;
		rawOdometry.right_wheel_rpm = 2.0f;

		testMessages.push_back(rawOdometry);
	}

	return testMessages;
}

vector<char> createPacket(OdometryRPM rawOdometry)
{
	vector<char> messageBuffer;

	HardwareMessage msg(messageBuffer);

	msg.write<uint8_t>(static_cast<uint8_t>(BRAIN_STEM_MSG::RAW_ODOMETRY));
	msg.write<uint32_t>(100);
	msg.write<float>(rawOdometry.left_wheel_rpm);
	msg.write<float>(rawOdometry.right_wheel_rpm);

	return std::move(messageBuffer);
}

namespace srslib_framework
{
	bool operator== (const OdometryRPM& state1,
		const OdometryRPM& state2)
	{
		return state1.left_wheel_rpm == state2.left_wheel_rpm &&
			state1.right_wheel_rpm == state2.right_wheel_rpm;
	}
}

TEST(Test_RawOdometryMsg, Key)
{
	RawOdometryHandler RawOdometryHandler;

    ASSERT_EQ(RawOdometryHandler.getKey(), BRAIN_STEM_MSG::RAW_ODOMETRY);
}

TEST(Test_RawOdometryMsg, EmptyPacket)
{
	RawOdometryHandler RawOdometryHandler;

	vector<char> messageBuffer;

	try
	{
		ASSERT_FALSE(RawOdometryHandler.receiveData(ros::Time(), messageBuffer));
	}
	catch(std::runtime_error&)
	{
		// Success
	}
}

TEST(Test_RawOdometryMsg, BadPacket)
{
	RawOdometryHandler RawOdometryHandler;

	vector<OdometryRPM>& testMsgs = getRawOdometryTestMsgs();

	vector<char> messageBuffer = createPacket(testMsgs[0]);
	messageBuffer.pop_back();

	try
	{
		ASSERT_FALSE(RawOdometryHandler.receiveData(ros::Time(), messageBuffer));
	}
	catch(std::runtime_error&)
	{
		// Success
	}
}

TEST(Test_RawOdometryMsg, ValidMessages)
{
	OdometryRPM RawOdometryMsg;

	RawOdometryHandler RawOdometryHandler([&](OdometryRPM& msg) {
		RawOdometryMsg = msg;
	});

	vector<OdometryRPM>& testMsgs = getRawOdometryTestMsgs();

	for (auto msg : testMsgs)
	{
		vector<char> messageBuffer = createPacket(testMsgs[0]);

		RawOdometryHandler.receiveData(ros::Time(), messageBuffer);

		ASSERT_EQ(RawOdometryMsg == msg, true);
	}
}
