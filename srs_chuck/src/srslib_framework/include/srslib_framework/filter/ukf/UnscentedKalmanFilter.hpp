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

#include <srslib_framework/platform/Ocv2Base.hpp>
#include <srslib_framework/filter/Process.hpp>
#include <srslib_framework/filter/FilterState.hpp>
#include <srslib_framework/filter/Command.hpp>
#include <srslib_framework/filter/Sensor.hpp>

namespace srs {

template<unsigned int STATE_SIZE = 5, unsigned int COMMAND_SIZE = 2, int TYPE = CV_64F>
class UnscentedKalmanFilter
{
public:
    typedef typename Ocv2Base<TYPE>::BaseType BaseType;

    UnscentedKalmanFilter(BaseType alpha, BaseType beta,
        Process<STATE_SIZE, COMMAND_SIZE, TYPE>& process);

    ~UnscentedKalmanFilter()
    {}

    void addSensor(Sensor<STATE_SIZE, TYPE>* sensor)
    {
        sensors_.push_back(sensor);
    }

    cv::Mat getState()
    {
        return state_;
    }

    cv::Mat getCovariance()
    {
        return covariance_;
    }

    void reset(cv::Mat stateT0, cv::Mat covarianceT0);
    void run(BaseType dT, Command<COMMAND_SIZE, TYPE>* const command);

private:
    constexpr static BaseType UNDERFLOW_THRESHOLD = BaseType(1.0e-5);

    vector<Sensor<STATE_SIZE, TYPE>*> sensors_;

    BaseType alpha_;
    BaseType beta_;
    BaseType kappa_;
    BaseType lambda_;
    BaseType c_;

    cv::Mat WM_;
    cv::Mat WC_;

    Process<STATE_SIZE, COMMAND_SIZE, TYPE>& process_;
    cv::Mat covariance_;
    cv::Mat state_;

    cv::Mat calculateSigmaPoints(cv::Mat M, cv::Mat P);

    void initializeWeights();

    void predict(BaseType dT, Command<COMMAND_SIZE, TYPE>* const command);

    void unscentedTransform(
        const cv::Mat X, const cv::Mat Y, const cv::Mat CHI,
        cv::Mat& Ybar, cv::Mat& S, cv::Mat& C);
    void update();
};

} // namespace srs

#include <srslib_framework/filter/ukf/UnscentedKalmanFilter.cpp>

#endif // UNSCENTEDKALMANFILTER_HPP_
