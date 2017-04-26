/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#include <sw_message/Test_SoftwareMessage.hpp>
#include <sw_message/PingHandler.hpp>

namespace srs {

class Test_Ping :
    public Test_SoftwareMessage<bool>
{
public:
    virtual ~Test_Ping() {}

	vector<char> createPacket(const bool& msg)
	{
		vector<char> messageBuffer;

		HardwareMessage hardwareMessage(messageBuffer);
		hardwareMessage.write<uint8_t>(static_cast<uint8_t>(BRAIN_STEM_CMD::PING));

		return std::move(messageBuffer);
	}
};


TEST_F(Test_Ping, TestEncodeData)
{
	PingHandler pingHandler(&messageProcessor_);

	bool ping = true;

	pingHandler.encodeData(ping);

	std::vector<char> message = createPacket(ping);

	ASSERT_EQ(messageProcessor_.command_, message);
}

} // namespace srs
