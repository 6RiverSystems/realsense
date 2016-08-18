#include <srsnode_motion/StatePe.hpp>

#include <srslib_framework/math/MatrixMath.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
template<unsigned int STATE_SIZE, int TYPE>
cv::Mat OdometrySensor<STATE_SIZE, TYPE>::getCurrentData()
{
    // Transfer the value that the odometer care about to the new state
    StatePe<TYPE> state = StatePe<TYPE>();

    state.velocity = currentData_.velocity;
    // state.velocity.linear = currentData_.velocity.linear;

    Sensor<STATE_SIZE, TYPE>::setNewData(false);

    return state.getVectorForm();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
template<unsigned int STATE_SIZE, int TYPE>
cv::Mat OdometrySensor<STATE_SIZE, TYPE>::H(const cv::Mat stateVector)
{
    StatePe<TYPE> state = StatePe<TYPE>(stateVector);

    // Transfer the value that the odometer care about to the new state
    cv::Mat result = MatrixMath::zeros(stateVector);
    result.at<BaseType>(StatePe<TYPE>::STATE_LINEAR) = state.velocity.linear;
    result.at<BaseType>(StatePe<TYPE>::STATE_ANGULAR) = state.velocity.angular;

    return result;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

} // namespace srs
