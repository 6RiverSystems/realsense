/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef LOGGING_HPP_
#define LOGGING_HPP_

#include <boost/algorithm/hex.hpp>

namespace srs {

	inline std::string ToHex( const std::vector<char>& data )
	{
	   std::ostringstream result;
	   result << std::setw(2) << std::setfill('0') << std::hex << std::showbase << std::uppercase;

	   std::copy( data.begin( ), data.end( ), std::ostream_iterator<unsigned int>(result, " "));

	   return result.str( );
	}

} // namespace srs

#endif // LOGGING_HPP_
