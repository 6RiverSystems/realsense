/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef COMMAND_HPP_
#define COMMAND_HPP_

#include <opencv2/opencv.hpp>

#include <srslib_framework/platform/Object.hpp>
#include <srslib_framework/platform/Ocv2Base.hpp>

namespace srs {

template<unsigned int COMMAND_SIZE = 2, int TYPE = CV_64F>
struct Command : public Object
{
    typedef typename Ocv2Base<TYPE>::BaseType BaseType;

    Command()
    {}

    ~Command()
    {}
};

} // namespace srs

#endif // COMMAND_HPP_
