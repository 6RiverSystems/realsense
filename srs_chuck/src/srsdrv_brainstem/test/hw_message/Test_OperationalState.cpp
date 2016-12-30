/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#include <hw_message/OperationalStateHandler.hpp>
#include <hw_message/Test_HardwareMessage.hpp>
#include <bitset>

class Test_OperationalState : public Test_HardwareMessage<const MsgOperationalState&, MsgOperationalState>
{
public:

	Test_OperationalState() :
		Test_HardwareMessage() {}

	virtual ~Test_OperationalState() {}

	vector<MsgOperationalState>& getTestMsgs()
	{
		static vector<MsgOperationalState> testMessages;

		if (!testMessages.size())
		{
			MsgOperationalState operationalState;
			operationalState.upTime = 1000;
			operationalState.frontEStop = false;
			operationalState.backEStop = false;
			operationalState.wirelessEStop = false;
			operationalState.bumpSensor = false;
			operationalState.pause = false;
			operationalState.hardStop = false;
			operationalState.safetyProcessorFailure = false;
			operationalState.brainstemFailure = false;
			operationalState.brainTimeoutFailure = false;
			operationalState.rightMotorFailure = false;
			operationalState.leftMotorFailure = false;

			testMessages.push_back(operationalState);
		}

		return testMessages;
	}

	vector<char> createPacket(const MsgOperationalState& operationalState, uint32_t version = 1)
	{
		vector<char> messageBuffer;

		HardwareMessage msg(messageBuffer);

		msg.write<uint8_t>(static_cast<uint8_t>(BRAIN_STEM_MSG::OPERATIONAL_STATE));
		msg.write<uint32_t>(operationalState.upTime);


		std::bitset<8> setMotionState;
		setMotionState.set( MOTION_STATUS::FRONT_E_STOP, operationalState.frontEStop );
		setMotionState.set( MOTION_STATUS::BACK_E_STOP, operationalState.backEStop );
		setMotionState.set( MOTION_STATUS::WIRELESS_E_STOP, operationalState.wirelessEStop );
		setMotionState.set( MOTION_STATUS::BUMP_SENSOR, operationalState.bumpSensor );
		setMotionState.set( MOTION_STATUS::FREE_SPIN, operationalState.pause );
		setMotionState.set( MOTION_STATUS::HARD_STOP, operationalState.hardStop );

		std::bitset<8> setFailureState;
		setFailureState.set( FAILURE_STATUS::SAFETY_PROCESSOR, operationalState.safetyProcessorFailure);
		setFailureState.set( FAILURE_STATUS::BRAINSTEM, operationalState.brainstemFailure );
		setFailureState.set( FAILURE_STATUS::BRAINSTEM_TIMEOUT, operationalState.brainTimeoutFailure );
		setFailureState.set( FAILURE_STATUS::RIGHT_MOTOR, operationalState.rightMotorFailure );
		setFailureState.set( FAILURE_STATUS::LEFT_MOTOR, operationalState.leftMotorFailure );

		msg.write<uint8_t>(setMotionState.to_ulong());
		msg.write<uint8_t>(setFailureState.to_ulong());

		return std::move(messageBuffer);
	}
};

namespace srslib_framework
{
	bool operator== (const MsgOperationalState& state1,
		const MsgOperationalState& state2)
	{
		return state1.upTime == state2.upTime &&
			state1.frontEStop == state2.frontEStop &&
			state1.backEStop == state2.backEStop &&
			state1.wirelessEStop == state2.wirelessEStop &&
			state1.bumpSensor == state2.bumpSensor &&
			state1.pause == state2.pause &&
			state1.hardStop == state2.hardStop &&
			state1.safetyProcessorFailure == state2.safetyProcessorFailure &&
			state1.brainstemFailure == state2.brainstemFailure &&
			state1.brainTimeoutFailure == state2.brainTimeoutFailure &&
			state1.rightMotorFailure == state2.rightMotorFailure &&
			state1.leftMotorFailure == state2.leftMotorFailure;
	}
}

TEST_F(Test_OperationalState, Key)
{
	OperationalStateHandler OperationalStateHandler(publisher_);

    ASSERT_EQ(OperationalStateHandler.getKey(), BRAIN_STEM_MSG::OPERATIONAL_STATE);
}

TEST_F(Test_OperationalState, EmptyPacket)
{
	OperationalStateHandler OperationalStateHandler(publisher_);

	vector<char> messageBuffer;

	try
	{
		OperationalStateHandler.receiveData(ros::Time(), messageBuffer);
	}
	catch(std::runtime_error&)
	{
		// Success
	}
}

TEST_F(Test_OperationalState, BadPacket)
{
	OperationalStateHandler OperationalStateHandler(publisher_);

	vector<MsgOperationalState>& testMsgs = getTestMsgs();

	vector<char> messageBuffer = createPacket(testMsgs[0]);
	messageBuffer.pop_back();

	try
	{
		OperationalStateHandler.receiveData(ros::Time(), messageBuffer);
	}
	catch(std::runtime_error&)
	{
		// Success
	}
}

TEST_F(Test_OperationalState, ValidMessage)
{
	OperationalStateHandler OperationalStateHandler(publisher_);

	vector<MsgOperationalState>& testMsgs = getTestMsgs();

	for (auto msg : testMsgs)
	{
		vector<char> messageBuffer = createPacket(msg);

		OperationalStateHandler.receiveData(ros::Time(), messageBuffer);

		// Copy value since version one did not have battery info
		EXPECT_EQ(publisher_.data_, msg);
	}
}
