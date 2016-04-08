#include "Robot.hpp"

#include <framework/Math.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
cv::Mat Robot::transform(const cv::Mat state, Command<>* const command, double DT)
{
    // x = x + v * DT * cos(theta);
    // y = y + v * DT * sin(theta);
    // theta = theta + omega * DT;
    double x = state.at<double>(FilterState<>::STATE_X);
    double y = state.at<double>(FilterState<>::STATE_Y);
    double theta = state.at<double>(FilterState<>::STATE_THETA);
    double v = state.at<double>(FilterState<>::STATE_V);
    double omega = state.at<double>(FilterState<>::STATE_OMEGA);

    x = x + v * DT * cos(theta);
    y = y + v * DT * sin(theta);
    theta = theta + omega * DT;

    // if command.valid
    //     v = command.velocity.linear;
    //     omega = command.velocity.angular;
    // end
    if (command)
    {
        v = command->getV();
        //v = command->getV();
        //omega = command->getOmega();
    }

    // Update the state to the next step
    cv::Mat result = Math::zeros(state);
    result.at<double>(FilterState<>::STATE_X) = x;
    result.at<double>(FilterState<>::STATE_Y) = y;
    result.at<double>(FilterState<>::STATE_THETA) = theta;
    result.at<double>(FilterState<>::STATE_V) = v;
    result.at<double>(FilterState<>::STATE_OMEGA) = omega;

    return result;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

} // namespace srs
