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

	srslib_framework::MsgOperationalState operationalStateMsg;

	operationalStateMsg.upTime = operationalState.upTime;

	operationalStateMsg.frontEStop = motionStatusSet.test( MOTION_STATUS::FRONT_E_STOP );
	operationalStateMsg.backEStop = motionStatusSet.test( MOTION_STATUS::BACK_E_STOP );
	operationalStateMsg.wirelessEStop = motionStatusSet.test( MOTION_STATUS::WIRELESS_E_STOP );
	operationalStateMsg.bumpSensor = motionStatusSet.test( MOTION_STATUS::BUMP_SENSOR);
	operationalStateMsg.pause = motionStatusSet.test(MOTION_STATUS::FREE_SPIN);
	operationalStateMsg.hardStop = motionStatusSet.test( MOTION_STATUS::HARD_STOP );

	operationalStateMsg.safetyProcessorFailure = failureStatusSet.test( FAILURE_STATUS::SAFETY_PROCESSOR );
	operationalStateMsg.brainstemFailure = failureStatusSet.test( FAILURE_STATUS::BRAINSTEM );
	operationalStateMsg.brainTimeoutFailure = failureStatusSet.test( FAILURE_STATUS::BRAINSTEM_TIMEOUT );
	operationalStateMsg.rightMotorFailure = failureStatusSet.test( FAILURE_STATUS::RIGHT_MOTOR );
	operationalStateMsg.leftMotorFailure = failureStatusSet.test( FAILURE_STATUS::LEFT_MOTOR );

	std::ostringstream stream;

	stream << "Operational State => uptime: " << operationalStateMsg.upTime <<
		", frontEStop: " << (operationalStateMsg.frontEStop ? "true" : "false")  <<
		", backEStop: " << (operationalStateMsg.backEStop ? "true" : "false")  <<
		", wirelessEStop: " << (operationalStateMsg.wirelessEStop ? "true" : "false")  <<
		", bumpSensor: " << (operationalStateMsg.bumpSensor ? "true" : "false")  <<
		", free-spin: " << (operationalStateMsg.pause ? "true" : "false")  <<
		", hardStop: " << (operationalStateMsg.hardStop ? "true" : "false")  <<
		", safetyProcessorFailure: " << (operationalStateMsg.safetyProcessorFailure ? "true" : "false")  <<
		", brainstemFailure: " << (operationalStateMsg.brainstemFailure ? "true" : "false")  <<
		", brainTimeoutFailure: " << (operationalStateMsg.brainTimeoutFailure ? "true" : "false")  <<
		", rightMotorFailure: " << (operationalStateMsg.rightMotorFailure ? "true" : "false")  <<
		", leftMotorFailure: " << (operationalStateMsg.leftMotorFailure ? "true" : "false")  <<
		std::endl;

	std::string strData =  stream.str( );

	ROS_INFO_STREAM( strData );

	publisher_.publish(operationalStateMsg);
}

} // namespace srs
