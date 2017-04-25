/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
using namespace std;

#include <ros/ros.h>

namespace srs {

template<typename TYPE>
class SingleDataSource
{
public:
    virtual ~SingleDataSource() {}

    virtual TYPE peek() const = 0;
    virtual TYPE pop() = 0;

    virtual void reset() = 0;

    virtual void set(TYPE data) = 0;
};

} // namespace srs
