/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#include <sw_message/Test_SoftwareMessage.hpp>
#include <sw_message/SetOdometryRpmHandler.hpp>

namespace srs {

class Test_SetOdometryRpm :
    public Test_SoftwareMessage<srslib_framework::OdometryRpm>
{
public:
    virtual ~Test_SetOdometryRpm() {}

    vector<srslib_framework::OdometryRpm>& getTestMsgs()
	{
		static vector<srslib_framework::OdometryRpm> testMessages;

		if (!testMessages.size())
		{
			srslib_framework::OdometryRpm odometryRpm;
			odometryRpm.left_wheel_rpm = 100;
			odometryRpm.right_wheel_rpm = 200;

			testMessages.push_back(odometryRpm);
		}

		return testMessages;
	}

	vector<char> createPacket(const srslib_framework::OdometryRpm& msg)
	{
		vector<char> messageBuffer;

		HardwareMessage hardwareMessage(messageBuffer);
		hardwareMessage.write<uint8_t>(static_cast<uint8_t>(BRAIN_STEM_CMD::SET_VELOCITY_RPM));
		hardwareMessage.write<float>(msg.left_wheel_rpm);
		hardwareMessage.write<float>(msg.right_wheel_rpm);

		return std::move(messageBuffer);
	}
};

TEST_F(Test_SetOdometryRpm, TestEncodeData)
{
	SetOdometryRpmHandler setOdometryRpmHandler(&messageProcessor_);

	vector<srslib_framework::OdometryRpm>& testMsgs = getTestMsgs();

	for (auto msg : testMsgs)
	{
		setOdometryRpmHandler.encodeData(msg);

		std::vector<char> message = createPacket(msg);

		ASSERT_EQ(messageProcessor_.command_, message);
	}
}

} // namespace srs
