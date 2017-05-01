/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#include <sw_message/Test_SoftwareMessage.hpp>
#include <sw_message/UpdateToteLightsHandler.hpp>

namespace srs {

class Test_UpdateToteLights :
    public Test_SoftwareMessage<srslib_framework::MsgUpdateToteLights>
{
public:
    virtual ~Test_UpdateToteLights() {}

	vector<srslib_framework::MsgUpdateToteLights>& getTestMsgs()
	{
		static vector<srslib_framework::MsgUpdateToteLights> testMessages;

		if (!testMessages.size())
		{
			// add a message to update tote light
			srslib_framework::MsgUpdateToteLights updateToteLights;

			updateToteLights.startSegment.x = static_cast<uint8_t>(0);
			updateToteLights.startSegment.y = static_cast<uint8_t>(0);
			updateToteLights.startSegment.z = static_cast<uint8_t>(0);
			updateToteLights.endSegment.x = static_cast<uint8_t>(10);
			updateToteLights.endSegment.y = static_cast<uint8_t>(1);
			updateToteLights.endSegment.z = static_cast<uint8_t>(1);
			updateToteLights.lightCmd = static_cast<uint8_t>(3);
			updateToteLights.startColor.r = static_cast<float>(255.0);
			updateToteLights.startColor.g = static_cast<float>(255.0);
			updateToteLights.startColor.b = static_cast<float>(255.0);
			updateToteLights.endColor.r = static_cast<uint8_t>(0.0);
			updateToteLights.endColor.g = static_cast<uint8_t>(0.0);
			updateToteLights.endColor.b = static_cast<uint8_t>(0.0);
			updateToteLights.frequency = static_cast<float>(10);

			testMessages.push_back(updateToteLights);
		}

		return testMessages;
	}

	vector<char> createPacket(const srslib_framework::MsgUpdateToteLights& msg)
	{
		vector<char> messageBuffer;

		HardwareMessage hardwareMessage(messageBuffer);
		hardwareMessage.write<uint8_t>(static_cast<uint8_t>(BRAIN_STEM_CMD::SET_TOTE_LIGHTS));
		hardwareMessage.write<uint8_t>(msg.startSegment.x);
		hardwareMessage.write<uint8_t>(msg.startSegment.y);
		hardwareMessage.write<uint8_t>(msg.startSegment.z);
		hardwareMessage.write<uint8_t>(msg.endSegment.x);
		hardwareMessage.write<uint8_t>(msg.endSegment.y);
		hardwareMessage.write<uint8_t>(msg.endSegment.z);
		hardwareMessage.write<uint8_t>(msg.lightCmd);
		hardwareMessage.write<uint8_t>(msg.startColor.r);
		hardwareMessage.write<uint8_t>(msg.startColor.g);
		hardwareMessage.write<uint8_t>(msg.startColor.b);
		hardwareMessage.write<uint8_t>(msg.endColor.r);
		hardwareMessage.write<uint8_t>(msg.endColor.g);
		hardwareMessage.write<uint8_t>(msg.endColor.b);
		hardwareMessage.write<float>(msg.frequency);

		return std::move(messageBuffer);
	}
};


TEST_F(Test_UpdateToteLights, TestEncodeData)
{
	UpdateToteLightsHandler UpdateToteLightsHandler(&messageProcessor_);

	vector<srslib_framework::MsgUpdateToteLights>& testMsgs = getTestMsgs();

	for (auto msg : testMsgs)
	{
		UpdateToteLightsHandler.encodeData(msg);

		std::vector<char> message = createPacket(msg);

		ASSERT_EQ(messageProcessor_.command_, message);
	}
}

} // namespace srs
