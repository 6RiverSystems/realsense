/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#include <srsdrv_brainstem/filters/PowerStateFilter.hpp>
#include <hw_message/Test_HardwareMessage.hpp>
#include <srslib_framework/ros/message/PowerStateMessageFactory.hpp>

class Test_PowerStateFilter : public ::testing::Test
{
public:
	typedef std::function<const BatteryState&(srslib_framework::MsgBatteryState)> ConvertFn;

	Test_PowerStateFilter() : publisher_(PowerStateMessageFactory::batteryState2Msg) {}

	virtual ~Test_PowerStateFilter() {}

	MockPublisher<const BatteryState&, MsgBatteryState> publisher_;

};

TEST_F(Test_PowerStateFilter, OneBattery)
{
	PowerState powerState;
	BatteryState batteryState1;
	BatteryState batteryState2;
	BatteryState batteryStateExpected;

	std::map<BatteryState::Descriptor, float> mapFiltered;

	batteryState1.descriptors[BatteryState::Descriptor::TEMPERATURE] =  210.0;
	batteryState2.descriptors[BatteryState::Descriptor::TEMPERATURE] =  50.0;
	batteryStateExpected.descriptors[BatteryState::Descriptor::TEMPERATURE] =  210.0;

	batteryState1.descriptors[BatteryState::Descriptor::VOLTAGE] =  24.0;
	batteryState2.descriptors[BatteryState::Descriptor::VOLTAGE] =  20.0;
	batteryStateExpected.descriptors[BatteryState::Descriptor::VOLTAGE] =  20.0;

	batteryState1.descriptors[BatteryState::Descriptor::AVERAGE_CURRENT] =  4.0;
	batteryState2.descriptors[BatteryState::Descriptor::AVERAGE_CURRENT] =  6.0;
	batteryStateExpected.descriptors[BatteryState::Descriptor::AVERAGE_CURRENT] = 5.0;

	batteryState1.descriptors[BatteryState::Descriptor::INSTANTANEOUS_CURRENT] =  1.0;
	batteryState2.descriptors[BatteryState::Descriptor::INSTANTANEOUS_CURRENT] =  3.0;
	batteryStateExpected.descriptors[BatteryState::Descriptor::INSTANTANEOUS_CURRENT] = 1.0;

	batteryState1.descriptors[BatteryState::Descriptor::CHARGED_PERCENTAGE] =  100.0;
	batteryState2.descriptors[BatteryState::Descriptor::CHARGED_PERCENTAGE] =  80.0;
	batteryStateExpected.descriptors[BatteryState::Descriptor::CHARGED_PERCENTAGE] =  80.0;

	batteryState1.descriptors[BatteryState::Descriptor::AVERAGE_TIME_TO_EMPTY] =  60.0;
	batteryState2.descriptors[BatteryState::Descriptor::AVERAGE_TIME_TO_EMPTY] =  40.0;
	batteryStateExpected.descriptors[BatteryState::Descriptor::AVERAGE_TIME_TO_EMPTY] =  50.0;

	powerState.batteries.push_back(batteryState1);
	powerState.batteries.push_back(batteryState2);

	PowerStateFilter powerstateFilter(publisher_);

	powerstateFilter.filter(powerState);

	BatteryState batteryState = PowerStateMessageFactory::msg2BatteryState(publisher_.data_);

	EXPECT_EQ(batteryState, batteryStateExpected);
}

TEST_F(Test_PowerStateFilter, OneDescriptor)
{
	PowerState powerState;
	BatteryState batteryState1;
	BatteryState batteryState2;
	BatteryState batteryStateExpected;

	std::map<BatteryState::Descriptor, float> mapFiltered;

	batteryState1.descriptors[BatteryState::Descriptor::CHARGED_PERCENTAGE] =  100.0;
	batteryState2.descriptors[BatteryState::Descriptor::CHARGED_PERCENTAGE] =  80.0;
	batteryStateExpected.descriptors[BatteryState::Descriptor::CHARGED_PERCENTAGE] =  80.0;

	powerState.batteries.push_back(batteryState1);
	powerState.batteries.push_back(batteryState2);

	PowerStateFilter powerstateFilter(publisher_);

	powerstateFilter.filter(powerState);

	BatteryState batteryState = PowerStateMessageFactory::msg2BatteryState(publisher_.data_);

	EXPECT_EQ(batteryState, batteryStateExpected);
}

TEST_F(Test_PowerStateFilter, Nan)
{
	PowerState powerState;
	BatteryState batteryState1;
	BatteryState batteryState2;
	BatteryState batteryStateExpected;

	std::map<BatteryState::Descriptor, float> mapFiltered;

	batteryState1.descriptors[BatteryState::Descriptor::CHARGED_PERCENTAGE] =  0.0;
	batteryStateExpected.descriptors[BatteryState::Descriptor::CHARGED_PERCENTAGE] =  0.0;

	powerState.batteries.push_back(batteryState1);
	powerState.batteries.push_back(batteryState2);

	PowerStateFilter powerstateFilter(publisher_);

	powerstateFilter.filter(powerState);

	BatteryState batteryState = PowerStateMessageFactory::msg2BatteryState(publisher_.data_);

	EXPECT_EQ(batteryState, batteryStateExpected);
}

