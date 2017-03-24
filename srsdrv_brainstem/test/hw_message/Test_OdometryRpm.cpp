/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#include <hw_message/OdometryRpmHandler.hpp>
#include <hw_message/Test_HardwareMessage.hpp>

class Test_OdometryRpm : public Test_HardwareMessage<const OdometryRpm&, OdometryRpm>
{
public:

	Test_OdometryRpm() :
		Test_HardwareMessage() {}

	virtual ~Test_OdometryRpm() {}

	vector<OdometryRpm>& getTestMsgs()
	{
		static vector<OdometryRpm> testMessages;

		if (!testMessages.size())
		{
			OdometryRpm rawOdometry;
			rawOdometry.left_wheel_rpm = 1.0f;
			rawOdometry.right_wheel_rpm = 2.0f;

			testMessages.push_back(rawOdometry);
		}

		return testMessages;
	}

	vector<char> createPacket(const OdometryRpm& rawOdometry, uint32_t version = 1)
	{
		vector<char> messageBuffer;

		HardwareMessage msg(messageBuffer);

		msg.write<uint8_t>(static_cast<uint8_t>(BRAIN_STEM_MSG::RAW_ODOMETRY));
		msg.write<uint32_t>(100);
		msg.write<float>(rawOdometry.left_wheel_rpm);
		msg.write<float>(rawOdometry.right_wheel_rpm);

		return std::move(messageBuffer);
	}
};

namespace srslib_framework
{
	bool operator== (const OdometryRpm& state1,
		const OdometryRpm& state2)
	{
		return state1.left_wheel_rpm == state2.left_wheel_rpm &&
			state1.right_wheel_rpm == state2.right_wheel_rpm;
	}
}


TEST_F(Test_OdometryRpm, Key)
{
	OdometryRpmHandler RawOdometryHandler(publisher_);

    ASSERT_EQ(RawOdometryHandler.getKey(), BRAIN_STEM_MSG::RAW_ODOMETRY);
}

TEST_F(Test_OdometryRpm, EmptyPacket)
{
	OdometryRpmHandler RawOdometryHandler(publisher_);

	vector<char> messageBuffer;

	try
	{
		RawOdometryHandler.receiveData(ros::Time(), messageBuffer);

		ASSERT_FALSE("Parsed invalid (empty) Rpm packet");
	}
	catch(std::runtime_error&)
	{
		// Success
	}
}

TEST_F(Test_OdometryRpm, BadPacket)
{
	OdometryRpmHandler RawOdometryHandler(publisher_);

	vector<OdometryRpm>& testMsgs = getTestMsgs();

	vector<char> messageBuffer = createPacket(testMsgs[0]);
	messageBuffer.pop_back();

	try
	{
		RawOdometryHandler.receiveData(ros::Time(), messageBuffer);

		ASSERT_FALSE("Parsed invalid (badly formed) Rpm packet");
	}
	catch(std::runtime_error&)
	{
		// Success
	}
}

TEST_F(Test_OdometryRpm, ValidMessages)
{
	OdometryRpmHandler RawOdometryHandler(publisher_);

	vector<OdometryRpm>& testMsgs = getTestMsgs();

	for (auto msg : testMsgs)
	{
		vector<char> messageBuffer = createPacket(testMsgs[0]);

		// Setup test expectations
		RawOdometryHandler.receiveData(ros::Time(), messageBuffer);

		EXPECT_EQ(publisher_.data_, msg);
	}
}
