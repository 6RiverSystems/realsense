/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef UNSCENTEDKALMANFILTER_HPP_
#define UNSCENTEDKALMANFILTER_HPP_

#include <vector>
using namespace std;

#include <opencv2/opencv.hpp>

#include <srslib_framework/utils/Ocv2Base.hpp>
#include <srslib_framework/filter/BaseKalmanFilter.hpp>
#include <srslib_framework/filter/Process.hpp>
#include <srslib_framework/filter/FilterState.hpp>
#include <srslib_framework/filter/Command.hpp>
#include <srslib_framework/filter/Sensor.hpp>

namespace srs {

template<unsigned int STATE_SIZE = 5, unsigned int COMMAND_SIZE = 2, int TYPE = CV_64F>
class UnscentedKalmanFilter : public BaseKalmanFilter<STATE_SIZE, COMMAND_SIZE, TYPE>
{
public:
    typedef BaseKalmanFilter<STATE_SIZE, COMMAND_SIZE, TYPE> BaseKFType;
    typedef typename BaseKFType::BaseType BaseType;

    UnscentedKalmanFilter(Process<STATE_SIZE, COMMAND_SIZE, TYPE>& process,
        BaseType alpha = 0.5, BaseType beta = 2.0);

    ~UnscentedKalmanFilter()
    {}

protected:

    void predict(BaseType dT, Command<COMMAND_SIZE, TYPE>* const command);

    void update();

private:
    constexpr static BaseType UNDERFLOW_THRESHOLD = BaseType(1.0e-5);

    BaseType alpha_;
    BaseType beta_;
    BaseType kappa_;
    BaseType lambda_;
    BaseType c_;

    cv::Mat WM_;
    cv::Mat WC_;

    cv::Mat calculateSigmaPoints(cv::Mat M, cv::Mat P);

    void initializeWeights();

    void unscentedTransform(const cv::Mat X, const cv::Mat Y, const cv::Mat CHI,
        cv::Mat& Ybar, cv::Mat& S, cv::Mat& C);
};

} // namespace srs

#include <filter/ukf/UnscentedKalmanFilter.cpp>

#endif // UNSCENTEDKALMANFILTER_HPP_
