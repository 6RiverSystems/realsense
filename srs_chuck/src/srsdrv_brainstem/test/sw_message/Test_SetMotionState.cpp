/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#include <sw_message/Test_SoftwareMessage.hpp>
#include <sw_message/SetMotionStateHandler.hpp>

namespace srs {

class Test_SetMotionState :
    public Test_SoftwareMessage<srslib_framework::MsgSetOperationalState>
{
public:
    virtual ~Test_SetMotionState() {}

	vector<srslib_framework::MsgSetOperationalState>& getTestMsgs()
	{
		static vector<srslib_framework::MsgSetOperationalState> testMessages;

		if (!testMessages.size())
		{
			srslib_framework::MsgSetOperationalState setOpState;
			setOpState.state = true;
			setOpState.operationalState.pause = false;

			testMessages.push_back(setOpState);
		}

		return testMessages;
	}

	vector<char> createPacket(const srslib_framework::MsgSetOperationalState& msg)
	{
		vector<char> messageBuffer;

		HardwareMessage hardwareMessage(messageBuffer);
		hardwareMessage.write<uint8_t>(static_cast<uint8_t>(BRAIN_STEM_CMD::SET_MOTION_STATUS));

		uint8_t motionState = 0;

		if (msg.operationalState.frontEStop)
		{
			motionState |= 0x1 << MOTION_STATUS::FRONT_E_STOP;
		}

		if (msg.operationalState.backEStop)
		{
			motionState |= 0x1 << MOTION_STATUS::BACK_E_STOP;
		}

		if (msg.operationalState.wirelessEStop)
		{
			motionState |= 0x1 << MOTION_STATUS::WIRELESS_E_STOP;
		}

		if (msg.operationalState.bumpSensor)
		{
			motionState |= 0x1 << MOTION_STATUS::BUMP_SENSOR;
		}

		if (msg.operationalState.pause)
		{
			motionState |= 0x1 << MOTION_STATUS::FREE_SPIN;
		}

		if (msg.operationalState.hardStop)
		{
			motionState |= 0x1 << MOTION_STATUS::HARD_STOP;
		}

		hardwareMessage.write<uint8_t>(motionState);

		return std::move(messageBuffer);
	}
};


TEST_F(Test_SetMotionState, TestEncodeData)
{
	SetMotionStateHandler setMotionStateHandler(&messageProcessor_);

	vector<srslib_framework::MsgSetOperationalState>& testMsgs = getTestMsgs();

	for (auto msg : testMsgs)
	{
		setMotionStateHandler.encodeData(msg);

		std::vector<char> message = createPacket(msg);

		ASSERT_EQ(messageProcessor_.command_, message);
	}
}

} // namespace srs
