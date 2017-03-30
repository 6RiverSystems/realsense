/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
using namespace std;

#include <srslib_framework/exception/SrsRuntimeErrorException.hpp>

namespace srs {

class InvalidChannelNumberException: public SrsRuntimeErrorException
{
public:
    InvalidChannelNumberException(const string& filename, int channels) :
        SrsRuntimeErrorException("The image must have Mono, RGB, or RGBA channels [" +
            filename + ", " + to_string(channels) + "]")
    {}
};

} // namespace srs
