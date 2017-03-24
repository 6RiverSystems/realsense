/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <stdexcept>
using namespace std;

namespace srs {

class SrsLogicErrorException: public logic_error
{
public:
    SrsLogicErrorException(const string& what) :
        logic_error(what)
    {}
};

} // namespace srs
