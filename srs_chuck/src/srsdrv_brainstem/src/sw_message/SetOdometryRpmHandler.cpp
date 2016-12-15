/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#include <BrainStemMessageProcessorInterface.hpp>

#include <sw_message/SetOdometryRpmHandler.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
SetOdometryRpmHandler::SetOdometryRpmHandler(BrainStemMessageProcessorInterface* owner) :
    SoftwareMessageHandler(owner)
{

}

////////////////////////////////////////////////////////////////////////////////////////////////////
void SetOdometryRpmHandler::attach()
{
	tapOdometryRpm_.reset(new TapBrainstemCmd_OdometryRpm());

	tapOdometryRpm_->attach(this);
}

void SetOdometryRpmHandler::notified(Subscriber<srslib_framework::OdometryRpm>* subject)
{
	TapBrainstemCmd_OdometryRpm* tap = static_cast<TapBrainstemCmd_OdometryRpm*>(subject);

	srslib_framework::OdometryRpm odometryRpm = tap->pop();

	encodeData(odometryRpm);
}

void SetOdometryRpmHandler::encodeData(const srslib_framework::OdometryRpm& odometryRpm)
{
	RawOdometryData msg = {
		static_cast<uint8_t>( BRAIN_STEM_CMD::SET_VELOCITY_RPM ),
		static_cast<float>( odometryRpm.left_wheel_rpm ),
		static_cast<float>( odometryRpm.right_wheel_rpm )
	};

	static double s_leftWheelRPM = odometryRpm.left_wheel_rpm;
	static double s_rightWheelRPM = odometryRpm.right_wheel_rpm;

	if( odometryRpm.left_wheel_rpm != s_leftWheelRPM ||
		odometryRpm.right_wheel_rpm != s_rightWheelRPM )
	{
		ROS_DEBUG_NAMED( "velocity_rpm", "Odometry: SetRPM: %f, %f",
			odometryRpm.left_wheel_rpm, odometryRpm.right_wheel_rpm );

		s_leftWheelRPM = odometryRpm.left_wheel_rpm;
		s_rightWheelRPM = odometryRpm.right_wheel_rpm;
	}

	getOwner()->sendCommand(reinterpret_cast<char*>(&msg), sizeof(msg));
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods


} // namespace srs
