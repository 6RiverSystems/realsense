/*
 * Logging.cpp
 *
 *  Created on: Jul 14, 2016
 *      Author: dan
 */

#include <srslib_framework/utils/Logging.hpp>

namespace srs
{

std::string ToHex( const std::vector<char>& data )
{
	std::string strDebug;

	for( char c : data )
	{
		char pszFormat[64] = { '\0' };
		snprintf( pszFormat, 64, " 0x%.2x", (unsigned char)c );
		strDebug += pszFormat;
	}

	return strDebug;
}

std::string string_format( const std::string fmt_str, ... )
{
	int final_n, n = ((int) fmt_str.size( )) * 2; /* Reserve two times as much as the length of the fmt_str */
	std::string str;
	std::unique_ptr<char[]> formatted;
	va_list ap;
	while( 1 )
	{
		formatted.reset( new char[n] ); /* Wrap the plain char array into the unique_ptr */
		strcpy( &formatted[0], fmt_str.c_str( ) );
		va_start( ap, fmt_str );
		final_n = vsnprintf( &formatted[0], n, fmt_str.c_str( ), ap );
		va_end( ap );
		if( final_n < 0 || final_n >= n )
		{
			n += abs( final_n - n + 1 );
		}
		else
		{
			break;
		}
	}
	return std::string( formatted.get( ) );
}

}
