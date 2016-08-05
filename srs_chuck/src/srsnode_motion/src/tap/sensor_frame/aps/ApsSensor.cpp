#include <srsnode_motion/StatePe.hpp>

#include <srslib_framework/math/MatrixMath.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
template<unsigned int STATE_SIZE, int TYPE>
cv::Mat ApsSensor<STATE_SIZE, TYPE>::getCurrentData()
{
    Sensor<STATE_SIZE, TYPE>::setNewData(false);

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