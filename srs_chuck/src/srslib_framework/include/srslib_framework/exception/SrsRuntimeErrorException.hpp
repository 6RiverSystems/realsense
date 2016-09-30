/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <stdexcept>
using namespace std;

namespace srs {

class SrsRuntimeErrorException: public runtime_error
{
public:
    SrsRuntimeErrorException(const string& what) :
        runtime_error(what)
    {}
};

} // namespace srs
