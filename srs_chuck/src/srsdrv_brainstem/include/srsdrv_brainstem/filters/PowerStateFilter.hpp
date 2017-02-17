/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <BrainStemMessages.hpp>
#include <srslib_framework/ros/channel/ChannelBrainstemPowerStateFiltered.hpp>
#include <srslib_framework/robotics/device/PowerState.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <functional>
#include <math.h>

using namespace boost::accumulators;

typedef accumulator_set<float, features<tag::min, tag::mean> > double_acc;

namespace srs {

class PowerStateFilter
{
	class DescriptorFilter
	{
		public:

			DescriptorFilter() :
				valid_(false),
				accumulator_() {}

			virtual void insert(float value)
			{
				accumulator_(value);

				valid_ = true;
			}

			bool isValid() { return valid_; };

			virtual float getFilteredValue() const = 0;

		protected:

			bool valid_;

			double_acc accumulator_;
	};

	class MedianDescriptorFilter : public DescriptorFilter
	{
		public:

			virtual float getFilteredValue() const
			{
				return mean(accumulator_);
			}
	};

public:

	PowerStateFilter(ChannelBrainstemPowerStateFiltered::Interface& publisher) : publisher_(publisher) {}

    void filter(const PowerState& powerState)
    {
    	std::map<BatteryState::Descriptor, std::shared_ptr<DescriptorFilter>> descriptorFilters;
    	descriptorFilters[BatteryState::Descriptor::TEMPERATURE] = std::make_shared<MedianDescriptorFilter>();
    	descriptorFilters[BatteryState::Descriptor::VOLTAGE] = std::make_shared<MedianDescriptorFilter>();
    	descriptorFilters[BatteryState::Descriptor::AVERAGE_CURRENT] = std::make_shared<MedianDescriptorFilter>();
    	descriptorFilters[BatteryState::Descriptor::INSTANTANEOUS_CURRENT] = std::make_shared<MedianDescriptorFilter>();
    	descriptorFilters[BatteryState::Descriptor::CHARGED_PERCENTAGE] = std::make_shared<MedianDescriptorFilter>();
    	descriptorFilters[BatteryState::Descriptor::AVERAGE_TIME_TO_EMPTY] = std::make_shared<MedianDescriptorFilter>();

    	for(const auto& battery : powerState.batteries)
    	{
    		for(const auto& descriptor : battery.descriptors)
    		{
    			const auto& descriptorFilter = descriptorFilters.find(descriptor.first);
    			if (descriptorFilter != descriptorFilters.end())
    			{
    				// Add the value
    				descriptorFilter->second->insert(descriptor.second);
    			}
    		}
    	}

    	srslib_framework::MsgBatteryState filteredPowerState;

    	for(const auto& filterTuple : descriptorFilters)
    	{
    		auto descriptorId = filterTuple.first;
    		const auto& descriptorFilter = filterTuple.second;

    	  	// Average Current
			srslib_framework::MsgBatteryDescriptor batteryDescriptor;
			batteryDescriptor.id = static_cast<uint8_t>(descriptorId);
			batteryDescriptor.value = descriptorFilter->getFilteredValue();

			if (std::isfinite(batteryDescriptor.value))
			{
				ROS_ERROR("Battery Descriptor: %d, %f", batteryDescriptor.id, batteryDescriptor.value);

				filteredPowerState.descriptors.push_back(batteryDescriptor);
			}
    	}

        publisher_.publish(filteredPowerState);
    }

private:

    ChannelBrainstemPowerStateFiltered::Interface& publisher_;

};

} // namespace srs
