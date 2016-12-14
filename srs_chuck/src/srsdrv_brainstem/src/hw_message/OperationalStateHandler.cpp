#include <srsdrv_brainstem/hw_message/OperationalStateHandler.hpp>
#include <bitset>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

OperationalStateHandler::OperationalStateHandler(ChannelBrainstemOperationalState::Interface& publisher) :
    HardwareMessageHandler(BRAIN_STEM_MSG::OPERATIONAL_STATE),
	publisher_(publisher)
{

}

////////////////////////////////////////////////////////////////////////////////////////////////////
void OperationalStateHandler::receiveMessage(ros::Time currentTime, HardwareMessage& msg)
{
	OperationalStateData operationalState = msg.read<OperationalStateData>();

	std::bitset<8> motionStatusSet( operationalState.motionStatus );
	std::bitset<8> failureStatusSet( operationalState.failureStatus );

	operationalState_.upTime = operationalState.upTime;

	operationalState_.frontEStop = motionStatusSet.test( MOTION_STATUS::FRONT_E_STOP );
	operationalState_.backEStop = motionStatusSet.test( MOTION_STATUS::BACK_E_STOP );
	operationalState_.wirelessEStop = motionStatusSet.test( MOTION_STATUS::WIRELESS_E_STOP );
	operationalState_.bumpSensor = motionStatusSet.test( MOTION_STATUS::BUMP_SENSOR);
	operationalState_.pause = motionStatusSet.test(MOTION_STATUS::FREE_SPIN);
	operationalState_.hardStop = motionStatusSet.test( MOTION_STATUS::HARD_STOP );

	operationalState_.safetyProcessorFailure = failureStatusSet.test( FAILURE_STATUS::SAFETY_PROCESSOR );
	operationalState_.brainstemFailure = failureStatusSet.test( FAILURE_STATUS::BRAINSTEM );
	operationalState_.brainTimeoutFailure = failureStatusSet.test( FAILURE_STATUS::BRAINSTEM_TIMEOUT );
	operationalState_.rightMotorFailure = failureStatusSet.test( FAILURE_STATUS::RIGHT_MOTOR );
	operationalState_.leftMotorFailure = failureStatusSet.test( FAILURE_STATUS::LEFT_MOTOR );

	std::ostringstream stream;

	stream << "Operational State => uptime: " << operationalState_.upTime <<
		", frontEStop: " << (operationalState_.frontEStop ? "true" : "false")  <<
		", backEStop: " << (operationalState_.backEStop ? "true" : "false")  <<
		", wirelessEStop: " << (operationalState_.wirelessEStop ? "true" : "false")  <<
		", bumpSensor: " << (operationalState_.bumpSensor ? "true" : "false")  <<
		", free-spin: " << (operationalState_.pause ? "true" : "false")  <<
		", hardStop: " << (operationalState_.hardStop ? "true" : "false")  <<
		", safetyProcessorFailure: " << (operationalState_.safetyProcessorFailure ? "true" : "false")  <<
		", brainstemFailure: " << (operationalState_.brainstemFailure ? "true" : "false")  <<
		", brainTimeoutFailure: " << (operationalState_.brainTimeoutFailure ? "true" : "false")  <<
		", rightMotorFailure: " << (operationalState_.rightMotorFailure ? "true" : "false")  <<
		", leftMotorFailure: " << (operationalState_.leftMotorFailure ? "true" : "false")  <<
		std::endl;

	std::string strData =  stream.str( );

	ROS_INFO_STREAM( strData );

	publisher_.publish(operationalState_);
}

void OperationalStateHandler::setBrainstemTimeout(bool brainstemTimeout)
{
	if (operationalState_.brainstemTimeoutFailure != brainstemTimeout)
	{
		operationalState_.brainstemTimeoutFailure = brainstemTimeout;

		ROS_INFO_STREAM("Operational State => brainstemTimeoutFailure: " << brainstemTimeout);

		publisher_.publish(operationalState_);
	}
}


} // namespace srs
