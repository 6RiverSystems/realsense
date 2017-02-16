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

namespace srslib_framework
{
	extern bool operator== (const MsgBatteryDescriptor& descriptor1,
		const MsgBatteryDescriptor& descriptor2);

	extern bool operator== (const MsgBatteryState& state1,
		const MsgBatteryState& state2);

	extern bool operator== (const MsgPowerState& state1,
		const MsgPowerState& state2);
}

TEST_F(Test_PowerStateFilter, OneBattery)
{
	srslib_framework::MsgPowerState powerState;
	srslib_framework::MsgBatteryState batteryState1;
	srslib_framework::MsgBatteryState batteryState2;
	srslib_framework::MsgBatteryState batteryStateFiltered;
	srslib_framework::MsgBatteryDescriptor batteryDescriptor;

	std::map<uint32_t, float> mapFiltered;

	batteryDescriptor.id = static_cast<uint32_t>(BATTERY_DESCRIPTOR::TEMPERATURE);
	batteryDescriptor.value = 100.0;
	batteryState1.descriptors.push_back(batteryDescriptor);
	batteryDescriptor.value = 50.0;
	batteryState2.descriptors.push_back(batteryDescriptor);
	mapFiltered[batteryDescriptor.id] = 75.0;

	batteryDescriptor.id = static_cast<uint32_t>(BATTERY_DESCRIPTOR::VOLTAGE);
	batteryDescriptor.value = 22.0;
	batteryState1.descriptors.push_back(batteryDescriptor);
	batteryDescriptor.value = 20.0;
	batteryState2.descriptors.push_back(batteryDescriptor);
	mapFiltered[batteryDescriptor.id] = 21.0;

	batteryDescriptor.id = static_cast<uint32_t>(BATTERY_DESCRIPTOR::AVERAGE_CURRENT);
	batteryDescriptor.value = 4.0;
	batteryState1.descriptors.push_back(batteryDescriptor);
	batteryDescriptor.value = 6.0;
	batteryState2.descriptors.push_back(batteryDescriptor);
	mapFiltered[batteryDescriptor.id] = 5.0;

	batteryDescriptor.id = static_cast<uint32_t>(BATTERY_DESCRIPTOR::INSTANTANEOUS_CURRENT);
	batteryDescriptor.value = 8.0;
	batteryState1.descriptors.push_back(batteryDescriptor);
	batteryDescriptor.value = 4.0;
	batteryState2.descriptors.push_back(batteryDescriptor);
	mapFiltered[batteryDescriptor.id] = 6.0;

	batteryDescriptor.id = static_cast<uint32_t>(BATTERY_DESCRIPTOR::CHARGED_PERCENTAGE);
	batteryDescriptor.value = 100.0;
	batteryState1.descriptors.push_back(batteryDescriptor);
	batteryDescriptor.value = 80.0;
	batteryState2.descriptors.push_back(batteryDescriptor);
	mapFiltered[batteryDescriptor.id] = 90.0;

	batteryDescriptor.id = static_cast<uint32_t>(BATTERY_DESCRIPTOR::AVERAGE_TIME_TO_EMPTY);
	batteryDescriptor.value = 60.0;
	batteryState1.descriptors.push_back(batteryDescriptor);
	batteryDescriptor.value = 40.0;
	batteryState2.descriptors.push_back(batteryDescriptor);
	mapFiltered[batteryDescriptor.id] = 50.0;

	powerState.batteries.push_back(batteryState1);
	powerState.batteries.push_back(batteryState2);

	PowerStateFilter powerstateFilter(publisher_);

	powerstateFilter.filter(powerState);

	std::map<uint32_t, float> mapResultsFiltered;

	for(auto descriptor : publisher_.data_.descriptors)
	{
		mapResultsFiltered[descriptor.id] = descriptor.value;
	}

	EXPECT_EQ(mapResultsFiltered, mapFiltered);
}
