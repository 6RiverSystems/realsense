#include <srsnode_motion/StatePe.hpp>

#include <srslib_framework/math/MatrixMath.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Constant initialization

// Covariance vector of the robot process
template<unsigned int STATE_SIZE, int TYPE>
const cv::Mat ApsSensor<STATE_SIZE, TYPE>::R = (
    cv::Mat_<ApsSensor<STATE_SIZE, TYPE>::BaseType>(1, STATE_SIZE) <<
        ApsSensor<STATE_SIZE, TYPE>::ERROR_LOCATION * ApsSensor<STATE_SIZE, TYPE>::ERROR_LOCATION, // [m^2]
        ApsSensor<STATE_SIZE, TYPE>::ERROR_LOCATION * ApsSensor<STATE_SIZE, TYPE>::ERROR_LOCATION, // [m^2]
        ApsSensor<STATE_SIZE, TYPE>::ERROR_HEADING * ApsSensor<STATE_SIZE, TYPE>::ERROR_HEADING, // [rad^2]
        0.0,
        0.0
);

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
template<unsigned int STATE_SIZE, int TYPE>
cv::Mat ApsSensor<STATE_SIZE, TYPE>::getCurrentData()
{
    // Transfer the value that the odometer care about to the new state
    StatePe<TYPE> state = StatePe<TYPE>();
    state.pose = currentData_;

    return state.getVectorForm();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
template<unsigned int STATE_SIZE, int TYPE>
cv::Mat ApsSensor<STATE_SIZE, TYPE>::H(const cv::Mat stateVector)
{
    StatePe<TYPE> state = StatePe<TYPE>(stateVector);

    // Transfer the value that the odometer care about to the new state
    cv::Mat result = MatrixMath::zeros(stateVector);
    result.at<BaseType>(StatePe<TYPE>::STATE_X) = state.pose.x;
    result.at<BaseType>(StatePe<TYPE>::STATE_Y) = state.pose.y;
    result.at<BaseType>(StatePe<TYPE>::STATE_THETA) = state.pose.theta;

    return result;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

} // namespace srs
