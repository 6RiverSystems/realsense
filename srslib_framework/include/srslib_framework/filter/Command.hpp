/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/math/Ocv2Base.hpp>

namespace srs {

template<unsigned int COMMAND_SIZE = 2, int TYPE = CV_64F>
struct Command
{
    typedef typename Ocv2Base<TYPE>::BaseType BaseType;

    Command()
    {}

    ~Command()
    {}
};

} // namespace srs
