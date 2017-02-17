/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

namespace srs {

struct BatteryState
{
	enum class Descriptor
	{
		TEMPERATURE				= 0x08,
		VOLTAGE					= 0x9,
		AVERAGE_CURRENT			= 0xB,
		INSTANTANEOUS_CURRENT	= 0xA,
		CHARGED_PERCENTAGE		= 0xD,
		AVERAGE_TIME_TO_EMPTY	= 0x12,
		INVALID					= 0xFF
	};

    BatteryState() :
    	descriptors()
    {}

    virtual ~BatteryState()
    {}

    static std::string getDescriptorDescription(const Descriptor& descriptor)
    {
        static std::map<Descriptor, std::string> validDescriptors_;

        if(!validDescriptors_.size())
        {
        	validDescriptors_[Descriptor::TEMPERATURE] = "Temperature (deg C)";
        	validDescriptors_[Descriptor::AVERAGE_CURRENT] = "Voltage (V)";
        	validDescriptors_[Descriptor::INSTANTANEOUS_CURRENT] = "Average current (A)";
        	validDescriptors_[Descriptor::CHARGED_PERCENTAGE] = "Instantaneous current (A)";
        	validDescriptors_[Descriptor::AVERAGE_TIME_TO_EMPTY] = "Percent charged (0-1)";
        }

    	if (validDescriptors_.find(descriptor) != validDescriptors_.end())
    	{
    	    return validDescriptors_[descriptor];
    	}
    	else
    	{
    		throw std::runtime_error("Invalid battery descriptor");
    	}
    }

    friend bool operator==(const BatteryState& lhs, const BatteryState& rhs)
    {
        return lhs.descriptors == rhs.descriptors;
    }

    friend std::ostream& operator<<(std::ostream& stream, const BatteryState& BatteryState)
    {
        stream << "{";

        for(auto descriptor: BatteryState.descriptors)
        {
        	auto descriptorId = descriptor.first;
            stream << getDescriptorDescription(descriptorId) << ": " << descriptor.second << " ";
        }

        stream << "}" << std::endl;

        return stream;
    }

    std::map<Descriptor, float> descriptors;

};

} // namespace srs
