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

#include <hw_message/SensorFrameHandler.hpp>

struct CombinedSensorMsg
{
	srslib_framework::SensorFrame sensorFrame;
	srslib_framework::Odometry odometry;
};

vector<CombinedSensorMsg>& getTestMsgs()
{
	static vector<CombinedSensorMsg> testMessages;

	if (!testMessages.size())
	{
		CombinedSensorMsg combinedMsg;

		testMessages.push_back(combinedMsg);
	}

	return testMessages;
}

vector<char> createPacket(CombinedSensorMsg combinedMsg)
{
	vector<char> messageBuffer;

	HardwareMessage msg(messageBuffer);

	msg.write<uint8_t>(static_cast<uint8_t>(BRAIN_STEM_MSG::SENSOR_FRAME));
	msg.write<uint32_t>(100);
	msg.write<float>(combinedMsg.sensorFrame.odometry.linear);
	msg.write<float>(combinedMsg.sensorFrame.odometry.angular);
	msg.write<uint32_t>(combinedMsg.odometry.left_wheel);
	msg.write<uint32_t>(combinedMsg.odometry.left_wheel);
	msg.write<float>(combinedMsg.sensorFrame.imu.yaw);
	msg.write<float>(combinedMsg.sensorFrame.imu.pitch);
	msg.write<float>(combinedMsg.sensorFrame.imu.roll);
	msg.write<float>(combinedMsg.sensorFrame.imu.yawRot);
	msg.write<float>(combinedMsg.sensorFrame.imu.pitchRot);
	msg.write<float>(combinedMsg.sensorFrame.imu.rollRot);

	return std::move(messageBuffer);
}

namespace srslib_framework
{
	bool operator == (const srslib_framework::Velocity& state1,
		const srslib_framework::Velocity& state2)
	{
		return state1.linear == state2.linear &&
			state1.angular == state2.angular;
	}

	bool operator == (const srslib_framework::Odometry& state1,
		const srslib_framework::Odometry& state2)
	{
		return state1.left_wheel == state2.left_wheel &&
			state1.right_wheel == state2.right_wheel;
	}

	bool operator == (const srslib_framework::Imu& state1,
		const srslib_framework::Imu& state2)
	{
		return state1.yaw == state2.yaw &&
			state1.pitch == state2.pitch &&
			state1.roll == state2.roll &&
			state1.yawRot == state2.yawRot &&
			state1.pitchRot == state2.pitchRot &&
			state1.rollRot == state2.rollRot;
	}

	bool operator == (const srslib_framework::SensorFrame& state1,
		const srslib_framework::SensorFrame& state2)
	{
		return state1.odometry == state2.odometry &&
			state1.imu == state2.imu;
	}
}

TEST(Test_SensorFrameMsg, Key)
{
	SensorFrameHandler SensorFrameHandler;

    ASSERT_EQ(SensorFrameHandler.getKey(), BRAIN_STEM_MSG::SENSOR_FRAME);
}

TEST(Test_SensorFrameMsg, EmptyPacket)
{
	SensorFrameHandler SensorFrameHandler;

	vector<char> messageBuffer;

	try
	{
		ASSERT_FALSE(SensorFrameHandler.receiveData(ros::Time(), messageBuffer));
	}
	catch(std::runtime_error&)
	{
		// Success
	}
}

TEST(Test_SensorFrameMsg, BadPacket)
{
	SensorFrameHandler SensorFrameHandler;

	vector<CombinedSensorMsg>& testMsgs = getTestMsgs();

	vector<char> messageBuffer = createPacket(testMsgs[0]);
	messageBuffer.pop_back();

	try
	{
		ASSERT_FALSE(SensorFrameHandler.receiveData(ros::Time(), messageBuffer));
	}
	catch(std::runtime_error&)
	{
		// Success
	}
}

TEST(Test_SensorFrameMsg, InvalidDescriptor)
{
	SensorFrameHandler SensorFrameHandler;

	vector<CombinedSensorMsg>& testMsgs = getTestMsgs();

	CombinedSensorMsg msg = testMsgs[0];

	vector<char> messageBuffer = createPacket(msg);

	try
	{
		ASSERT_FALSE(SensorFrameHandler.receiveData(ros::Time(), messageBuffer));
	}
	catch(std::runtime_error&)
	{
		// Success
	}
}

TEST(Test_SensorFrameMsg, ValidMessages)
{
	srslib_framework::Imu imuMsg;

	CombinedSensorMsg combinedMsg;

	SensorFrameHandler SensorFrameHandler([&](srslib_framework::Imu& msg) {
		imuMsg = msg;
	}, [&](srslib_framework::Odometry& msg) {
		combinedMsg.odometry = msg;
	}, [&](srslib_framework::SensorFrame& msg) {
		combinedMsg.sensorFrame = msg;
	});

	vector<CombinedSensorMsg>& testMsgs = getTestMsgs();

	for (auto msg : testMsgs)
	{
		vector<char> messageBuffer = createPacket(testMsgs[0]);

		SensorFrameHandler.receiveData(ros::Time::now(), messageBuffer);

		ASSERT_EQ(combinedMsg.odometry == msg.odometry, true);
		ASSERT_EQ(combinedMsg.sensorFrame == msg.sensorFrame, true);
		ASSERT_EQ(combinedMsg.sensorFrame.imu == imuMsg, true);
	}
}
