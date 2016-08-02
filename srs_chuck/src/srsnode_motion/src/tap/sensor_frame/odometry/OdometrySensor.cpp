#include <srsnode_motion/StatePe.hpp>

#include <srslib_framework/math/MatrixMath.hpp>

namespace srs {

//////////////////////////////////////////////////////////////////////////////////////////////////////
//// Constant initialization
//
//// Covariance vector of the odometry sensor
//template<unsigned int STATE_SIZE, int TYPE>
//const cv::Mat OdometrySensor<STATE_SIZE, TYPE>::R = (
//    cv::Mat_<OdometrySensor<STATE_SIZE, TYPE>::BaseType>(1, STATE_SIZE) <<
//        0,
//        0,
//        0,
//        OdometrySensor<STATE_SIZE, TYPE>::ERROR_LINEAR_VELOCITY *
//            OdometrySensor<STATE_SIZE, TYPE>::ERROR_LINEAR_VELOCITY, // [m^2/s^2]
//        OdometrySensor<STATE_SIZE, TYPE>::ERROR_LINEAR_VELOCITY *
//            OdometrySensor<STATE_SIZE, TYPE>::ERROR_LINEAR_VELOCITY // [m^2/s^2]
//);

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
template<unsigned int STATE_SIZE, int TYPE>
cv::Mat OdometrySensor<STATE_SIZE, TYPE>::getCurrentData()
{
    // Transfer the value that the odometer care about to the new state
    StatePe<TYPE> state = StatePe<TYPE>();
    state.velocity = currentData_.velocity;

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
