/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef VELCMD_HPP_
#define VELCMD_HPP_

#include <string>
using namespace std;

#include <opencv2/opencv.hpp>

#include <srslib_framework/platform/Object.hpp>
#include <srslib_framework/platform/Ocv2Base.hpp>
#include <srslib_framework/filter/Command.hpp>

#include "Configuration.hpp"

namespace srs {

template<int TYPE = CV_64F>
struct VelCmd : public Command<STATIC_UKF_COMMAND_VECTOR_SIZE, TYPE>
{
    typedef typename Ocv2Base<TYPE>::BaseType BaseType;

    VelCmd(const BaseType v, const BaseType omega) :
        Command<STATIC_UKF_COMMAND_VECTOR_SIZE, TYPE>(),
        v(v),
        omega(omega)
    {}

    VelCmd() :
        Command<STATIC_UKF_COMMAND_VECTOR_SIZE, TYPE>(),
        v(BaseType()),
        omega(BaseType())
    {}

    ~VelCmd()
    {}

    string toString()
    {
        ostringstream output;
        output << "VelCmd {" << endl;
        output << "      v: " << v << endl;
        output << "  omega: " << omega << endl;
        output << "}" << endl;

        return output.str();
    }

    BaseType v;
    BaseType omega;
};

} // namespace srs

#endif // VELCMD_HPP_
