/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef RUN11STEPSDATA_HPP_
#define RUN11STEPSDATA_HPP_

namespace srs {

const static cv::Mat COV_STEP_01 = (cv::Mat_<double>(UKF_STATE_SIZE, UKF_STATE_SIZE) <<
    +2.0000e-04,             0,             0,             0,             0,
              0,   +3.0000e-04,             0,             0,             0,
              0,             0,   +3.7453e-04,             0,             0,
              0,             0,             0,   +9.9900e-05,             0,
              0,             0,             0,             0,   +9.9900e-05
);

const static cv::Mat COV_STEP_02 = (cv::Mat_<double>(UKF_STATE_SIZE, UKF_STATE_SIZE) <<
    +6.7439e-04,             0,   -3.7446e-04,             0,             0,
              0,   +4.9987e-04,             0,   +9.9701e-05,             0,
    -3.7446e-04,             0,   +6.4887e-04,             0,   +9.9701e-05,
              0,   +9.9701e-05,             0,   +1.9950e-04,             0,
              0,             0,   +9.9701e-05,             0,   +1.9950e-04
);

const static cv::Mat COV_STEP_03 = (cv::Mat_<double>(UKF_STATE_SIZE, UKF_STATE_SIZE) <<
    +2.1718e-03,             0,   -1.1226e-03,             0,   -9.9381e-05,
              0,   +9.9795e-04,             0,   +2.9831e-04,             0,
    -1.1226e-03,             0,   +1.2214e-03,             0,   +2.9831e-04,
              0,   +2.9831e-04,             0,   +2.9861e-04,             0,
    -9.9381e-05,             0,   +2.9831e-04,             0,   +2.9861e-04
);

const static cv::Mat COV_STEP_04 = (cv::Mat_<double>(UKF_STATE_SIZE, UKF_STATE_SIZE) <<
    +5.7354e-03,             0,   -2.7389e-03,             0,   -3.9602e-04,
              0,   +1.9898e-03,             0,   +5.9455e-04,             0,
    -2.7389e-03,             0,   +2.2876e-03,             0,   +5.9455e-04,
              0,   +5.9455e-04,             0,   +3.9702e-04,             0,
    -3.9602e-04,             0,   +5.9455e-04,             0,   +3.9702e-04
);

const static cv::Mat COV_STEP_05 = (cv::Mat_<double>(UKF_STATE_SIZE, UKF_STATE_SIZE) <<
    +1.3585e-02,             0,   -6.0056e-03,             0,   -9.8534e-04,
              0,   +3.6668e-03,             0,   +9.8667e-04,             0,
    -6.0056e-03,             0,   +4.0385e-03,             0,   +9.8667e-04,
              0,   +9.8667e-04,             0,   +4.9457e-04,             0,
    -9.8534e-04,             0,   +9.8667e-04,             0,   +4.9457e-04
);

const static cv::Mat COV_STEP_06 = (cv::Mat_<double>(UKF_STATE_SIZE, UKF_STATE_SIZE) <<
    +2.9671e-02,             0,   -1.1982e-02,             0,   -1.9594e-03,
              0,   +6.2156e-03,             0,   +1.4725e-03,             0,
    -1.1982e-02,             0,   +6.6591e-03,             0,   +1.4725e-03,
              0,   +1.4725e-03,             0,   +5.9105e-04,             0,
    -1.9594e-03,             0,   +1.4725e-03,             0,   +5.9105e-04
);

const static cv::Mat COV_STEP_07 = (cv::Mat_<double>(UKF_STATE_SIZE, UKF_STATE_SIZE) <<
    +6.0192e-02,             0,   -2.1986e-02,             0,   -3.4058e-03,
              0,   +9.8182e-03,             0,   +2.0494e-03,             0,
    -2.1986e-02,             0,   +1.0327e-02,             0,   +2.0494e-03,
              0,   +2.0494e-03,             0,   +6.8631e-04,             0,
    -3.4058e-03,             0,   +2.0494e-03,             0,   +6.8631e-04
);

const static cv::Mat COV_STEP_08 = (cv::Mat_<double>(UKF_STATE_SIZE, UKF_STATE_SIZE) <<
    +1.1405e-01,             0,   -3.7580e-02,             0,   -5.4067e-03,
              0,   +1.4655e-02,             0,   +2.7143e-03,             0,
    -3.7580e-02,             0,   +1.5213e-02,             0,   +2.7143e-03,
              0,   +2.7143e-03,             0,   +7.8017e-04,             0,
    -5.4067e-03,             0,   +2.7143e-03,             0,   +7.8017e-04
);

const static cv::Mat COV_STEP_09 = (cv::Mat_<double>(UKF_STATE_SIZE, UKF_STATE_SIZE) <<
    +2.0325e-01,             0,   -6.0540e-02,             0,   -8.0380e-03,
              0,   +2.0905e-02,             0,   +3.4640e-03,             0,
    -6.0540e-02,             0,   +2.1475e-02,             0,   +3.4640e-03,
              0,   +3.4640e-03,             0,   +8.7250e-04,             0,
    -8.0380e-03,             0,   +3.4640e-03,             0,   +8.7250e-04
);

const static cv::Mat COV_STEP_10 = (cv::Mat_<double>(UKF_STATE_SIZE, UKF_STATE_SIZE) <<
    +3.4318e-01,             0,   -9.2833e-02,             0,   -1.1368e-02,
              0,   +2.8756e-02,             0,   +4.2947e-03,             0,
    -9.2833e-02,             0,   +2.9264e-02,             0,   +4.2947e-03,
              0,   +4.2947e-03,             0,   +9.6313e-04,             0,
    -1.1368e-02,             0,   +4.2947e-03,             0,   +9.6313e-04
);

const static cv::Mat COV_STEP_11 = (cv::Mat_<double>(UKF_STATE_SIZE, UKF_STATE_SIZE) <<
    +5.5524e-01,             0,   -1.3740e-01,             0,             0,
              0,   +3.8676e-02,             0,             0,             0,
    -1.3740e-01,             0,   +3.8991e-02,             0,             0,
              0,             0,             0,   +9.9900e-05,             0,
              0,             0,             0,             0,   +9.9900e-05
);

} // namespace srs

#endif // RUN11STEPSDATA_HPP_
