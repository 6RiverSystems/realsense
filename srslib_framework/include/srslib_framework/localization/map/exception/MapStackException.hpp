/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/exception/SrsRuntimeErrorException.hpp>

namespace srs {

class MapStackException: public SrsRuntimeErrorException
{
public:
    MapStackException(const string& filename, const string& what) :
        SrsRuntimeErrorException(filename + ": " + what)
    {}
};

} // namespace srs
