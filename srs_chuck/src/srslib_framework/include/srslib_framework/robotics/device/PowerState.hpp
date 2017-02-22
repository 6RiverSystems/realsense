/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/robotics/device/BatteryState.hpp>

namespace srs {

struct PowerState
{

    PowerState()
    {}

    virtual ~PowerState()
    {}

    friend bool operator==(const PowerState& lhs, const PowerState& rhs)
    {
        return lhs.batteries == rhs.batteries;
    }

    friend std::ostream& operator<<(std::ostream& stream, const PowerState& powerState)
    {
    	stream << "PowerState {";

    	uint8_t index = 1;

		for(auto battery : powerState.batteries)
		{
			stream << "battery" << index++ << ": " << battery << ", ";
		}

		stream << "}" << std::endl;

        return stream;
    }

    std::list<BatteryState> batteries;
};

} // namespace srs
