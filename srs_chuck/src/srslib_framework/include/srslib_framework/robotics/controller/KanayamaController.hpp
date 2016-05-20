/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef KANAYAMACONTROLLER_HPP_
#define KANAYAMACONTROLLER_HPP_

#include <opencv2/opencv.hpp>

#include <srslib_framework/math/PoseMath.hpp>
#include <srslib_framework/robotics/Pose.hpp>
#include <srslib_framework/robotics/Velocity.hpp>

#include <srslib_test/utils/Print.hpp>

namespace srs {

class KanayamaController
{
public:
    KanayamaController(double Kx, double Ky, double Kt) :
        Kx_(Kx),
        Ky_(Ky),
        Kt_(Kt),
        command_()
    {}

    ~KanayamaController()
    {}

    Velocity<> getCommand() const
    {
        return command_;
    }

    void step(Pose<> cPose)
    {
        calculateE(cPose);

        double vb = Kx_ * E_.at<double>(0);
        double wb = rVelocity_.linear * (Ky_ * E_.at<double>(1) + Kt_ * sin(E_.at<double>(2)));

        command_.linear = rVelocity_.linear * cos(E_.at<double>(2)) + vb;
        command_.angular = rVelocity_.angular + wb;
    }

    void setReference(Pose<> pose, Velocity<> velocity)
    {
        Qr_ = PoseMath::pose2mat<>(pose);
        rVelocity_ = velocity;
    }

private:

    void calculateE(Pose<>& cPose)
    {
        cv::Mat Qc = PoseMath::pose2mat<>(cPose);

        double c = cos(cPose.theta);
        double s = sin(cPose.theta);

        cv::Mat T = (cv::Mat_<double>(3, 3) <<
             c, s, 0,
            -s, c, 0,
             0, 0, 1);

        E_ = T * (Qr_ - Qc);
        test::Print::print(E_, "E");
    }

    cv::Mat E_;

    Velocity<> command_;

    double Kx_;
    double Ky_;
    double Kt_;

    cv::Mat Qr_;

    Velocity<> rVelocity_;
};

} // namespace srs

#endif // KANAYAMACONTROLLER_HPP_
