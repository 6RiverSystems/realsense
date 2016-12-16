/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/exception/SrsRuntimeErrorException.hpp>
#include <srslib_framework/search/AStar.hpp>
#include <srslib_framework/search/Plan.hpp>

namespace srs {

class UnexpectedSearchActionException: public SrsRuntimeErrorException
{
public:
    UnexpectedSearchActionException(Plan& plan) :
        SrsRuntimeErrorException("Unexpected valid search action in this plan " +
            plan.toString())
    {}
};

} // namespace srs
