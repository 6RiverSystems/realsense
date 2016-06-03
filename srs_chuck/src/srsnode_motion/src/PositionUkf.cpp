#include <srsnode_motion/PositionUkf.hpp>

#include <srsnode_motion/StatePe.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
// protected methods

//////////////////////////////////////////////////////////////////////////////////////////////////////
cv::Mat PositionUkf::averageTransform(const cv::Mat W, const cv::Mat X)
{
    cout << "active ##########################################################" << endl;

    double c = 0.0;
    double s = 0.0;

    cv::Mat R = Math::zeros(X);

    for (unsigned int i = 0; i < X.cols; ++i)
    {
        double weight = W.at<BaseType>(i);
        R += weight * X.col(i);

        double theta = X.at<BaseType>(i, StatePe<>::STATE_THETA);
        c +=  weight * cos(theta);
        s +=  weight * sin(theta);
    }

    R.at<BaseType>(StatePe<>::STATE_THETA) = atan2(s, c);

    return R;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
cv::Mat PositionUkf::residualTransform(const cv::Mat A, const cv::Mat B)
{
    cv::Mat R = A - B;

    double theta = A.at<BaseType>(StatePe<>::STATE_THETA);
    double c = cos(theta);
    double s = sin(theta);

    theta = B.at<BaseType>(StatePe<>::STATE_THETA);
    c +=  cos(theta);
    s +=  sin(theta);

    R.at<BaseType>(StatePe<>::STATE_THETA) = atan2(s, c);

    return R;
}

} // namespace srs
