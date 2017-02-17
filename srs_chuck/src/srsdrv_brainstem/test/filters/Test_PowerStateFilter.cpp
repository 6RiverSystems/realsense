/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#include <srsdrv_brainstem/filters/PowerStateFilter.hpp>
#include <hw_message/Test_HardwareMessage.hpp>

class Test_PowerStateFilter : public ::testing::Test
{
public:
	typedef std::function<const srslib_framework::MsgBatteryState&(srslib_framework::MsgBatteryState)> ConvertFn;

	Test_PowerStateFilter() : publisher_([](const srslib_framework::MsgBatteryState& data) {
		srslib_framework::MsgBatteryState msg;
		msg.descriptors = data.descriptors;
		return msg;
	}) {}



	virtual ~Test_PowerStateFilter() {}

	MockPublisher<const srslib_framework::MsgBatteryState&, srslib_framework::MsgBatteryState> publisher_;

};

TEST_F(Test_PowerStateFilter, OneBattery)
{
	PowerState powerState;
	BatteryState batteryState1;
	BatteryState batteryState2;
	BatteryState batteryStateFiltered;

	std::map<uint32_t, float> mapFiltered;

	batteryState1.descriptors[BatteryState::Descriptor::TEMPERATURE] =  100.0;
	batteryState2.descriptors[BatteryState::Descriptor::TEMPERATURE] =  50.0;
	batteryStateFiltered.descriptors[BatteryState::Descriptor::TEMPERATURE] =  75.0;

	batteryState1.descriptors[BatteryState::Descriptor::VOLTAGE] =  24.0;
	batteryState2.descriptors[BatteryState::Descriptor::VOLTAGE] =  20.0;
	batteryStateFiltered.descriptors[BatteryState::Descriptor::VOLTAGE] =  22.0;

	batteryState1.descriptors[BatteryState::Descriptor::AVERAGE_CURRENT] =  4.0;
	batteryState2.descriptors[BatteryState::Descriptor::AVERAGE_CURRENT] =  6.0;
	batteryStateFiltered.descriptors[BatteryState::Descriptor::AVERAGE_CURRENT] = 5.0;

	batteryState1.descriptors[BatteryState::Descriptor::INSTANTANEOUS_CURRENT] =  1.0;
	batteryState2.descriptors[BatteryState::Descriptor::INSTANTANEOUS_CURRENT] =  3.0;
	batteryStateFiltered.descriptors[BatteryState::Descriptor::INSTANTANEOUS_CURRENT] = 2.0;

	batteryState1.descriptors[BatteryState::Descriptor::CHARGED_PERCENTAGE] =  100.0;
	batteryState2.descriptors[BatteryState::Descriptor::CHARGED_PERCENTAGE] =  80.0;
	batteryStateFiltered.descriptors[BatteryState::Descriptor::CHARGED_PERCENTAGE] =  90.0;

	batteryState1.descriptors[BatteryState::Descriptor::AVERAGE_TIME_TO_EMPTY] =  60.0;
	batteryState2.descriptors[BatteryState::Descriptor::AVERAGE_TIME_TO_EMPTY] =  40.0;
	batteryStateFiltered.descriptors[BatteryState::Descriptor::AVERAGE_TIME_TO_EMPTY] =  50.0;

	powerState.batteries.push_back(batteryState1);
	powerState.batteries.push_back(batteryState2);

	PowerStateFilter powerstateFilter(publisher_);

	powerstateFilter.filter(powerState);

	EXPECT_EQ(publisher_.data_, batteryStateFiltered);
}
