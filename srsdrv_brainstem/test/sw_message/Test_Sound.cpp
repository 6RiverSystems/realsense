/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#include <sw_message/Test_SoftwareMessage.hpp>
#include <sw_message/SoundHandler.hpp>

namespace srs {

class Test_Sound :
    public Test_SoftwareMessage<Sound>
{
public:
    virtual ~Test_Sound() {}

	vector<Sound>& getTestMsgs()
	{
		static vector<Sound> testMessages;

		if (!testMessages.size())
		{
			Sound sound(128, 1024, 100, 50, 10);

			testMessages.push_back(sound);
		}

		return testMessages;
	}

	vector<char> createPacket(const Sound& msg)
	{
		vector<char> messageBuffer;

		HardwareMessage hardwareMessage(messageBuffer);
		hardwareMessage.write<uint8_t>(static_cast<uint8_t>(BRAIN_STEM_CMD::SOUND_BUZZER));
		hardwareMessage.write<uint8_t>(msg.volume);
		hardwareMessage.write<uint16_t>(msg.baseFrequency);
		hardwareMessage.write<uint16_t>(msg.cycleRate);
		hardwareMessage.write<uint8_t>(msg.dutyCycle);
		hardwareMessage.write<uint16_t>(msg.numberOfCycles);

		return std::move(messageBuffer);
	}
};


TEST_F(Test_Sound, TestEncodeData)
{
	SoundHandler soundHandler(&messageProcessor_);

	vector<Sound>& testMsgs = getTestMsgs();

	for (auto msg : testMsgs)
	{
		soundHandler.encodeData(msg);

		std::vector<char> message = createPacket(msg);

		ASSERT_EQ(messageProcessor_.command_, message);
	}
}

} // namespace srs
