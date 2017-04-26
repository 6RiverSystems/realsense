/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef LOGGING_HPP_
#define LOGGING_HPP_

#include <boost/algorithm/hex.hpp>
#include <cstdio>
#include <stdarg.h>
#include <memory>

namespace srs
{

std::string ToHex( const std::vector<char>& data );

std::string string_format(const std::string fmt_str, ...);

} // namespace srs

#endif // LOGGING_HPP_
