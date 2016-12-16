/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#include <BrainStemMessageProcessorInterface.hpp>

#include <sw_message/SetVelocityHandler.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
SetVelocityHandler::SetVelocityHandler(BrainStemMessageProcessorInterface* owner) :
    SoftwareMessageHandler(owner)
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void SetVelocityHandler::attach()
{
	// TODO: Add to base subscriber class (delay ros intantiation for unit tests)
}

void SetVelocityHandler::receiveData(const geometry_msgs::Twist::ConstPtr data)
{
	SetVelocityData msg = {
		static_cast<uint8_t>( BRAIN_STEM_CMD::SET_VELOCITY ),
		static_cast<float>( data->linear.x ),
		static_cast<float>( data->angular.y )
	};

	getOwner()->sendCommand( reinterpret_cast<char*>( &msg ), sizeof(msg) );
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods


} // namespace srs
