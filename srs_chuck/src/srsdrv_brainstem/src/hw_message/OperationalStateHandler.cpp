#include <srsdrv_brainstem/hw_message/OperationalStateHandler.hpp>
#include <bitset>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

OperationalStateHandler::OperationalStateHandler(ChannelBrainstemOperationalState::Interface& publisher) :
    HardwareMessageHandler(BRAIN_STEM_MSG::OPERATIONAL_STATE),
    publisher_(publisher),
    hasValidMessage_(false)
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void OperationalStateHandler::receiveMessage(ros::Time currentTime, HardwareMessage& msg)
{
    OperationalStateData operationalState = msg.read<OperationalStateData>();

    std::bitset<8> motionStatusSet(operationalState.motionStatus);
    std::bitset<8> failureStatusSet(operationalState.failureStatus);

    operationalState_.upTime = operationalState.upTime;

    operationalState_.frontEStop = motionStatusSet.test(MOTION_STATUS::FRONT_E_STOP);
    operationalState_.backEStop = motionStatusSet.test(MOTION_STATUS::BACK_E_STOP);
    operationalState_.wirelessEStop = motionStatusSet.test(MOTION_STATUS::WIRELESS_E_STOP);
    operationalState_.bumpSensor = motionStatusSet.test(MOTION_STATUS::BUMP_SENSOR);
    operationalState_.freeSpin = motionStatusSet.test(MOTION_STATUS::FREE_SPIN);
    operationalState_.hardStop = motionStatusSet.test(MOTION_STATUS::HARD_STOP);

    operationalState_.safetyProcessorFailure = failureStatusSet.test(FAILURE_STATUS::SAFETY_PROCESSOR_FAILURE);
    operationalState_.brainstemFailure = failureStatusSet.test(FAILURE_STATUS::BRAINSTEM_FAILURE);
    operationalState_.brainstemTimeout = false;
    operationalState_.brainTimeout = failureStatusSet.test(FAILURE_STATUS::BRAIN_TIMEOUT);
    operationalState_.rightMotorController = failureStatusSet.test(FAILURE_STATUS::RIGHT_MOTOR_CONTROLLER);
    operationalState_.leftMotorController = failureStatusSet.test(FAILURE_STATUS::LEFT_MOTOR_CONTROLLER);
    operationalState_.rpmMessageTimeout = failureStatusSet.test(FAILURE_STATUS::RPM_MESSAGE_TIMEOUT);
    operationalState_.velocityViolation = failureStatusSet.test(FAILURE_STATUS::VELOCITY_VIOLATION);

    ROS_INFO_STREAM("Brainstem driver: " << operationalState_);

    publisher_.publish(operationalState_);

    hasValidMessage_ = true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void OperationalStateHandler::setBrainstemTimeout(bool brainstemTimeout)
{
    if (operationalState_.brainstemTimeout != brainstemTimeout)
    {
        operationalState_.brainstemTimeout = brainstemTimeout;

        ROS_INFO_STREAM("Brainstem driver: " << operationalState_);

        publisher_.publish(operationalState_);
    }
}


} // namespace srs
