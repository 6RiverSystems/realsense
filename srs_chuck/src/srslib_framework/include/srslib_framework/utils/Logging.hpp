/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef LOGGING_HPP_
#define LOGGING_HPP_

#include <boost/algorithm/hex.hpp>
#include <cstdio>

namespace srs
{

inline std::string ToHex( const std::vector<char>& data )
{
	std::string strDebug;

	for( char c : data )
	{
		if( std::isprint( c ) )
		{
			strDebug += c;
		}
		else
		{
			char pszFormat[64] = { '\0' };
		    snprintf( pszFormat, 64, " 0x%.2x", c );
		    strDebug += pszFormat;
		}
	}

	return strDebug;
}

} // namespace srs

#endif // LOGGING_HPP_
