/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#include <sw_message/Test_SoftwareMessage.hpp>
#include <sw_message/UpdateBodyLightsHandler.hpp>

namespace srs {

class Test_UpdateBodyLights :
    public Test_SoftwareMessage<srslib_framework::MsgUpdateBodyLights>
{
public:
    virtual ~Test_UpdateBodyLights() {}

	vector<srslib_framework::MsgUpdateBodyLights>& getTestMsgs()
	{
		static vector<srslib_framework::MsgUpdateBodyLights> testMessages;

		if (!testMessages.size())
		{
			// add message for left tail LED
			srslib_framework::MsgUpdateBodyLights updateTailLeftLight;

			srslib_framework::MsgBodyLightsState tailLeftLight;
			tailLeftLight.entity = static_cast<uint8_t>(BODY_LIGHTS_ENTITIES::TAIL_LEFT);
			tailLeftLight.lightCmd = static_cast<uint8_t>(LED_COMMAND::RAMP);
			tailLeftLight.startColor.r = static_cast<float>(255.0);
			tailLeftLight.startColor.g = static_cast<float>(255.0);
			tailLeftLight.startColor.b = static_cast<float>(255.0);
			tailLeftLight.endColor.r = static_cast<float>(0.0);
			tailLeftLight.endColor.g = static_cast<float>(0.0);
			tailLeftLight.endColor.b = static_cast<float>(0.0);
			tailLeftLight.frequency = static_cast<float>(10);

			updateTailLeftLight.bodyLightUpdates.push_back(tailLeftLight);
			testMessages.push_back(updateTailLeftLight);

			// add message for pause
			srslib_framework::MsgUpdateBodyLights updatePauseLight;

			srslib_framework::MsgBodyLightsState pauseLight;
			pauseLight.entity = static_cast<uint8_t>(BODY_LIGHTS_ENTITIES::PAUSE);
			pauseLight.lightCmd = static_cast<uint8_t>(LED_COMMAND::SOLID);
			pauseLight.startColor.r = static_cast<float>(255.0);
			pauseLight.startColor.g = static_cast<float>(0.0);
			pauseLight.startColor.b = static_cast<float>(0.0);
			pauseLight.endColor.r = static_cast<float>(0.0);
			pauseLight.endColor.g = static_cast<float>(0.0);
			pauseLight.endColor.b = static_cast<float>(0.0);
			pauseLight.frequency = static_cast<float>(10);

			updatePauseLight.bodyLightUpdates.push_back(pauseLight);
			testMessages.push_back(updatePauseLight);
		}

		return testMessages;
	}

	vector<char> createPacket(const srslib_framework::MsgUpdateBodyLights& msg)
	{
		vector<char> messageBuffer;

		HardwareMessage hardwareMessage(messageBuffer);

		for (auto uiElement : msg.bodyLightUpdates)
		{
			hardwareMessage.write<uint8_t>(static_cast<uint8_t>(BRAIN_STEM_CMD::SET_BODY_LIGHTS));
			hardwareMessage.write<uint8_t>(uiElement.entity);
			hardwareMessage.write<uint8_t>(uiElement.lightCmd);
			hardwareMessage.write<uint8_t>(uiElement.startColor.r);
			hardwareMessage.write<uint8_t>(uiElement.startColor.g);
			hardwareMessage.write<uint8_t>(uiElement.startColor.b);
			hardwareMessage.write<uint8_t>(uiElement.endColor.r);
			hardwareMessage.write<uint8_t>(uiElement.endColor.g);
			hardwareMessage.write<uint8_t>(uiElement.endColor.b);
			hardwareMessage.write<float>(uiElement.frequency);

			return std::move(messageBuffer);
		}
	}
};


TEST_F(Test_UpdateBodyLights, TestEncodeData)
{
	UpdateBodyLightsHandler UpdateBodyLightsHandler(&messageProcessor_);

	vector<srslib_framework::MsgUpdateBodyLights>& testMsgs = getTestMsgs();

	for (auto msg : testMsgs)
	{
		UpdateBodyLightsHandler.encodeData(msg);

		std::vector<char> message = createPacket(msg);

		ASSERT_EQ(messageProcessor_.command_, message);
	}
}

} // namespace srs
