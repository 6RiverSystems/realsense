#include <srslib_framework/robotics/robot/Chuck.hpp>

#include <srslib_framework/math/PoseMath.hpp>

#include <srsnode_motion/CmdVelocity.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
template<int TYPE>
cv::Mat Robot<TYPE>::FB(
    const cv::Mat stateVector,
    Command<STATIC_UKF_COMMAND_VECTOR_SIZE, TYPE>* const command,
    BaseType dT)
{
    StatePe<TYPE> oldState(stateVector);

    if (command)
    {
//        Velocity<> newVelocity = reinterpret_cast<CmdVelocity<>*>(command)->velocity;
//        Velocity<> oldVelocity = oldState.velocity;
//
//        // Calculate the average of the velocity using the
//        // velocity at the beginning and end of the time slice
//        oldState.velocity = Velocity<>(
//            (newVelocity.linear + oldVelocity.linear) / 2,
//            (newVelocity.angular + oldVelocity.angular) / 2);
        oldState.velocity = reinterpret_cast<CmdVelocity<>*>(command)->velocity;
    }

    StatePe<TYPE> newState;
    Robot<TYPE>::kinematics(oldState, dT, newState);

    return newState.getVectorForm();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
template<int TYPE>
void Robot<TYPE>::kinematics(StatePe<TYPE> sT0, BaseType dT, StatePe<TYPE>& sT1)
{
    double v = sT0.velocity.linear;
    double w = sT0.velocity.angular;

    // Check for the special case in which omega is 0 (the robot is moving straight)
    if (abs(w) > ANGULAR_VELOCITY_EPSILON)
    {
        double r = v / w;

        sT1.pose = Pose<>(
            sT0.pose.x + r * sin(sT0.pose.theta + w * dT) - r * sin(sT0.pose.theta),
            sT0.pose.y + r * cos(sT0.pose.theta) - r * cos(sT0.pose.theta + w * dT),
            sT0.pose.theta + w * dT);
    }
    else
    {
        sT1.pose = PoseMath::transform<double>(sT0.pose, v * dT);
    }

    sT1.velocity = sT0.velocity;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

} // namespace srs
