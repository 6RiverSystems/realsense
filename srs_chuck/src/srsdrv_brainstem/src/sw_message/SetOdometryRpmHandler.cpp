#include <srsdrv_brainstem/BrainStemMessageProcessor.h>

#include <ros/ros.h>

#include <sw_message/SetOdometryRpmHandler.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
SetOdometryRpmHandler::SetOdometryRpmHandler(BrainStemMessageProcessor* owner) :
    SoftwareMessageHandler(owner)
{
	tapOdometryRpm_.attach(this);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void SetOdometryRpmHandler::notified(Subscriber<srslib_framework::OdometryRpm>* subject)
{
	TapBrainstemCmd_OdometryRpm* tap = static_cast<TapBrainstemCmd_OdometryRpm*>(subject);

	srslib_framework::OdometryRpm odometryRpm = tap->pop();

	float& leftWheelRpm = odometryRpm.left_wheel_rpm;
	float& rightWheelRpm = odometryRpm.right_wheel_rpm;

	ODOMETRY_RPM_DATA msg = {
	    static_cast<uint8_t>( BRAIN_STEM_CMD::SET_VELOCITY_RPM ),
	    static_cast<float>( leftWheelRpm ),
	    static_cast<float>( rightWheelRpm )
	};

	static double s_leftWheelRPM = leftWheelRpm;
	static double s_rightWheelRPM = rightWheelRpm;

	if( leftWheelRpm != s_leftWheelRPM ||
		rightWheelRpm != s_rightWheelRPM )
	{
		ROS_DEBUG_NAMED( "velocity_rpm", "Odometry: SetRPM: %f, %f", leftWheelRpm, rightWheelRpm );

		s_leftWheelRPM = leftWheelRpm;
		s_rightWheelRPM = rightWheelRpm;
	}

	getOwner()->sendCommand(reinterpret_cast<char*>(&msg), sizeof(msg));
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods


} // namespace srs
