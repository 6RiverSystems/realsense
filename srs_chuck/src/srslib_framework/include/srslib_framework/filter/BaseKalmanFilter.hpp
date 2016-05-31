/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef BASEKALMANFILTER_HPP_
#define BASEKALMANFILTER_HPP_

#include <vector>
using namespace std;

#include <opencv2/opencv.hpp>

#include <srslib_framework/utils/Ocv2Base.hpp>
#include <srslib_framework/filter/Process.hpp>
#include <srslib_framework/filter/FilterState.hpp>
#include <srslib_framework/filter/Command.hpp>
#include <srslib_framework/filter/Sensor.hpp>

#include <srslib_test/utils/Print.hpp>

namespace srs {

template<unsigned int STATE_SIZE = 5, unsigned int COMMAND_SIZE = 2, int TYPE = CV_64F>
class BaseKalmanFilter
{
public:
    typedef typename Ocv2Base<TYPE>::BaseType BaseType;

    BaseKalmanFilter(Process<STATE_SIZE, COMMAND_SIZE, TYPE>& process) :
        sensors_(),
        process_(process)
    {}

    virtual ~BaseKalmanFilter()
    {}

    void addSensor(Sensor<STATE_SIZE, TYPE>* sensor)
    {
        sensors_.push_back(sensor);
    }

    cv::Mat getX()
    {
        return x_;
    }

    cv::Mat getP()
    {
        return P_;
    }

    void reset(cv::Mat x0, cv::Mat P0)
    {
        // Initialize the state of the filter with an adapted
        // version of the provided initial state and covariance
        x0.convertTo(x_, TYPE);
        P0.convertTo(P_, TYPE);
    }

    void run(BaseType dT, Command<COMMAND_SIZE, TYPE>* const command)
    {
        predict(dT, command);
        update();

        test::Print::print(x_, "x_");
        test::Print::print(P_, "P_");
    }

protected:

    virtual void predict(BaseType dT, Command<COMMAND_SIZE, TYPE>* const command) = 0;

    virtual void update() = 0;

    vector<Sensor<STATE_SIZE, TYPE>*> sensors_;

    Process<STATE_SIZE, COMMAND_SIZE, TYPE>& process_;
    cv::Mat P_;
    cv::Mat x_;
};

} // namespace srs

#endif // BASEKALMANFILTER_HPP_
