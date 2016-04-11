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

#include <platform/Ocv2Base.hpp>

#include <filter/Measurement.hpp>
#include <filter/Process.hpp>
#include <filter/FilterState.hpp>
#include <filter/Command.hpp>

namespace srs {

template<unsigned int STATE_SIZE = 5, int TYPE = CV_64F>
class UnscentedKalmanFilter
{
public:
    typedef typename Ocv2Base<TYPE>::BaseType BaseType;

    UnscentedKalmanFilter(BaseType alpha, BaseType beta,
        Process<STATE_SIZE, TYPE>& process,
        BaseType dT);
    ~UnscentedKalmanFilter();

    void reset(FilterState<TYPE> stateT0, cv::Mat covarianceT0);
    void run(Command<TYPE>* const command,
        const vector<Measurement<STATE_SIZE, TYPE>> measurements);

private:
    BaseType alpha_;
    BaseType beta_;
    BaseType kappa_;
    BaseType lambda_;
    BaseType c_;
    BaseType dT_;

    cv::Mat WM_;
    cv::Mat WC_;

    Process<STATE_SIZE, TYPE>& process_;
    cv::Mat covariance_;
    cv::Mat state_;

    cv::Mat calculateSigmaPoints(cv::Mat M, cv::Mat P);
    void checkCovarianceUnderflow(cv::Mat& S);

    void initializeWeights();

    void predict(Command<TYPE>* const command);

    void unscentedTransform(
        const cv::Mat XX, const cv::Mat Y, const cv::Mat CHI,
        cv::Mat& Ybar, cv::Mat& S, cv::Mat& C);
    void update(const vector<Measurement<STATE_SIZE, TYPE>> measurements);
};

} // namespace srs

#include "UnscentedKalmanFilter.cpp"

#endif // UNSCENTEDKALMANFILTER_HPP_
