/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef UNSCENTEDKALMANFILTER_HPP_
#define UNSCENTEDKALMANFILTER_HPP_

#include <opencv2/opencv.hpp>

#include <filter/Measurement.hpp>
#include <filter/ProcessModel.hpp>
#include <filter/FilterState.hpp>

namespace srs {

template<unsigned int STATE_SIZE = 5>
class UnscentedKalmanFilter
{
public:
    UnscentedKalmanFilter(double alpha, double beta, ProcessModel<>& processModel);
    ~UnscentedKalmanFilter();

    void reset(FilterState<STATE_SIZE> stateT0, cv::Mat covarianceT0);
    void run(Command<>* const command, const std::vector<Measurement> measurements);

private:
    double alpha_;
    double beta_;
    double kappa_;
    double lambda_;
    double c_;

    cv::Mat WM_;
    cv::Mat WC_;

    ProcessModel<>& processModel_;
    cv::Mat covariance_;
    cv::Mat state_;

    cv::Mat calculateSigmaPoints(cv::Mat M, cv::Mat P);

    void initializeWeights();

    void prediction(Command<>* const command);

    void unscentedTransform(
        const cv::Mat XX, const cv::Mat Y, const cv::Mat chi,
        cv::Mat& ybar, cv::Mat& S, cv::Mat& C);
    void update();
};

} // namespace srs

#include "UnscentedKalmanFilter.cpp"

#endif // UNSCENTEDKALMANFILTER_HPP_
