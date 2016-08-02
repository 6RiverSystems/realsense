/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef CMDVELOCITY_HPP_
#define CMDVELOCITY_HPP_

#include <string>
using namespace std;

#include <opencv2/opencv.hpp>

#include <srslib_framework/filter/Command.hpp>
#include <srslib_framework/math/Ocv2Base.hpp>
#include <srslib_framework/platform/Object.hpp>
#include <srslib_framework/robotics/Velocity.hpp>

#include <srsnode_motion/Configuration.hpp>

namespace srs {

template<int TYPE = CV_64F>
struct CmdVelocity : public Command<STATIC_UKF_COMMAND_VECTOR_SIZE, TYPE>
{
    typedef typename Ocv2Base<TYPE>::BaseType BaseType;

    CmdVelocity(const Velocity<BaseType> velocity) :
        Command<STATIC_UKF_COMMAND_VECTOR_SIZE, TYPE>(),
        velocity(velocity)
    {}

    CmdVelocity() :
        Command<STATIC_UKF_COMMAND_VECTOR_SIZE, TYPE>(),
        velocity()
    {}

    ~CmdVelocity()
    {}

    friend ostream& operator<<(ostream& stream, const CmdVelocity& cmdVelocity)
    {
        return "CmdVelocity {" << cmdVelocity.velocity << "}";
    }

    Velocity<BaseType> velocity;
};

} // namespace srs

#endif // CMDVELOCITY_HPP_
