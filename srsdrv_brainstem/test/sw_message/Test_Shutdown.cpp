/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#include <sw_message/Test_SoftwareMessage.hpp>
#include <sw_message/ShutdownHandler.hpp>

namespace srs {

class Test_Shutdown :
    public Test_SoftwareMessage<bool>
{
public:
    virtual ~Test_Shutdown() {}

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
		hardwareMessage.write<uint8_t>(static_cast<uint8_t>(BRAIN_STEM_CMD::SHUTDOWN));

		return std::move(messageBuffer);
	}
};


TEST_F(Test_Shutdown, TestEncodeData)
{
	ShutdownHandler shutdownHandler(&messageProcessor_);

	vector<bool>& testMsgs = getTestMsgs();

	for (auto msg : testMsgs)
	{
		shutdownHandler.encodeData(msg);

		std::vector<char> message = createPacket(msg);

		ASSERT_EQ(messageProcessor_.command_, message);
	}
}

} // namespace srs
