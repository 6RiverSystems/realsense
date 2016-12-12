/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#include <sw_message/Test_SoftwareMessage.hpp>
#include <sw_message/StartupHandler.hpp>

namespace srs {

class Test_Startup :
    public Test_SoftwareMessage<bool>
{
public:
    virtual ~Test_Startup() {}

    vector<bool>& getTestMsgs()
	{
		static vector<bool> testMessages;

		if (!testMessages.size())
		{
			testMessages.push_back(true);
		}

		return testMessages;
	}

	vector<char> createPacket(const bool& msg)
	{
		vector<char> messageBuffer;

		HardwareMessage hardwareMessage(messageBuffer);
		hardwareMessage.write<uint8_t>(static_cast<uint8_t>(BRAIN_STEM_CMD::STARTUP));

		return std::move(messageBuffer);
	}
};


TEST_F(Test_Startup, TestEncodeData)
{
	StartupHandler startupHandler(&messageProcessor_);

	vector<bool>& testMsgs = getTestMsgs();

	for (auto msg : testMsgs)
	{
		startupHandler.encodeData(msg);

		std::vector<char> message = createPacket(msg);

		ASSERT_EQ(messageProcessor_.command_, message);
	}
}

} // namespace srs
