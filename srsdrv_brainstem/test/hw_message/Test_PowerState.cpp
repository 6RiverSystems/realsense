/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#include <gtest/gtest.h>

#include <BrainStemMessages.hpp>

#include <srslib_test/utils/Compare.hpp>

#include <hw_message/PowerStateHandler.hpp>
#include <hw_message/Test_HardwareMessage.hpp>

#include <srslib_framework/ros/message/PowerStateMessageFactory.hpp>

using namespace srs;
using namespace srslib_framework;

class Test_PowerState : public Test_HardwareMessage<const PowerState&, MsgPowerState>
{
public:

	Test_PowerState() :
		Test_HardwareMessage(PowerStateMessageFactory::powerState2Msg) {}

	virtual ~Test_PowerState() {}

	vector<MsgPowerState>& getTestMsgs()
	{
		static vector<MsgPowerState> testMessages;

		if (!testMessages.size())
		{
			MsgBatteryDescriptor temperature;
			temperature.id = (int)BatteryState::Descriptor::TEMPERATURE;
			temperature.value = 100;

			MsgBatteryDescriptor voltage;
			voltage.id = (int)BatteryState::Descriptor::VOLTAGE;
			voltage.value = 22;

			MsgBatteryDescriptor current;
			current.id = (int)BatteryState::Descriptor::AVERAGE_CURRENT;
			current.value = 4;

			MsgBatteryDescriptor chargedPercentage;
			chargedPercentage.id = (int)BatteryState::Descriptor::CHARGED_PERCENTAGE;
			chargedPercentage.value = 50;

			MsgBatteryDescriptor timeToEmpty;
			timeToEmpty.id = (int)BatteryState::Descriptor::AVERAGE_TIME_TO_EMPTY;
			timeToEmpty.value = 60;


			MsgBatteryState batteryState;
			batteryState.descriptors.push_back(temperature);
			batteryState.descriptors.push_back(voltage);
			batteryState.descriptors.push_back(current);
			batteryState.descriptors.push_back(chargedPercentage);
			batteryState.descriptors.push_back(timeToEmpty);

			MsgPowerState powerState;
			powerState.batteries.push_back(batteryState);

			testMessages.push_back(powerState);
		}

		return testMessages;
	}

	vector<char> createPacket(const MsgPowerState& powerState, uint32_t version = 1)
	{
		uint8_t numberOfBatteries = powerState.batteries.size();
		uint8_t numberOfDescriptors = powerState.batteries[0].descriptors.size();

		vector<char> messageBuffer;

		HardwareMessage msg(messageBuffer);

		msg.write<uint8_t>(static_cast<uint8_t>(BRAIN_STEM_MSG::POWER_STATE));
		msg.write<uint8_t>(numberOfBatteries);
		msg.write<uint8_t>(numberOfDescriptors);

		for (int batteryIndex = 0; batteryIndex < numberOfBatteries; batteryIndex++)
		{
			const MsgBatteryState& batteryState = powerState.batteries[batteryIndex];

			for (int descriptorIndex = 0; descriptorIndex < numberOfDescriptors; descriptorIndex++)
			{
				const MsgBatteryDescriptor& descriptor = batteryState.descriptors[descriptorIndex];

				msg.write<uint8_t>(descriptor.id);
				msg.write<uint16_t>(descriptor.value);
			}
		}

		return std::move(messageBuffer);
	}
};

namespace srslib_framework
{
	bool operator== (const MsgBatteryDescriptor& descriptor1,
		const MsgBatteryDescriptor& descriptor2)
	{
		return (descriptor1.id == descriptor2.id) &&
			(descriptor1.value == descriptor2.value);
	}

	bool operator== (const MsgBatteryState& state1,
		const MsgBatteryState& state2)
	{
		return state1.descriptors == state2.descriptors;
	}

	bool operator== (const MsgPowerState& state1,
		const MsgPowerState& state2)
	{
		return state1.batteries == state2.batteries;
	}
}

TEST_F(Test_PowerState, Key)
{
	PowerStateHandler powerStateHandler(publisher_);

    ASSERT_EQ(powerStateHandler.getKey(), BRAIN_STEM_MSG::POWER_STATE);
}

TEST_F(Test_PowerState, EmptyPacket)
{
	PowerStateHandler powerStateHandler(publisher_);

	vector<char> messageBuffer;

	try
	{
		powerStateHandler.receiveData(ros::Time(), messageBuffer);

		ASSERT_FALSE("Parsed invalid (empty) Rpm packet");
	}
	catch(std::runtime_error&)
	{
		// Success
	}
}

TEST_F(Test_PowerState, BadPacket)
{
	PowerStateHandler PowerStateHandler(publisher_);

	vector<MsgPowerState>& testMsgs = getTestMsgs();

	vector<char> messageBuffer = createPacket(testMsgs[0]);
	messageBuffer.pop_back();

	try
	{
		PowerStateHandler.receiveData(ros::Time(), messageBuffer);

		ASSERT_FALSE("Parsed invalid (badly formed) packet");
	}
	catch(std::runtime_error&)
	{
		// Success
	}
}

TEST_F(Test_PowerState, InvalidDescriptor)
{
	PowerStateHandler powerStateHandler(publisher_);

	vector<MsgPowerState>& testMsgs = getTestMsgs();

	MsgPowerState powerState = testMsgs[0];

	MsgBatteryDescriptor invalid;
	invalid.id = INT8_MAX;
	invalid.value = 50;
	powerState.batteries[0].descriptors.push_back(invalid);

	vector<char> messageBuffer = createPacket(powerState);

	try
	{
		powerStateHandler.receiveData(ros::Time(), messageBuffer);

		ASSERT_FALSE("Parsed invalid (badly formed) descriptor");
	}
	catch(std::runtime_error&)
	{
		// Success
	}
}

TEST_F(Test_PowerState, ValidMessages)
{
	PowerStateHandler powerStateHandler(publisher_);

	vector<MsgPowerState>& testMsgs = getTestMsgs();

	for (auto msg : testMsgs)
	{
		vector<char> messageBuffer = createPacket(testMsgs[0]);

		// Setup test expectations
		powerStateHandler.receiveData(ros::Time(), messageBuffer);

		msg.batteries[0].descriptors[0].value = -263.15;
		msg.batteries[0].descriptors[1].value = 0.022;
		msg.batteries[0].descriptors[2].value = 4;
		msg.batteries[0].descriptors[3].value = 0.5;
		msg.batteries[0].descriptors[4].value = 60;

		EXPECT_EQ(publisher_.data_, msg);
	}
}
