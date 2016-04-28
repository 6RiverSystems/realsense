#include <string>
#include <boost/algorithm/hex.hpp>

std::string ToHex( const std::vector<char>& data )
{
   std::ostringstream result;
   result << std::setw(2) << std::setfill('0') << std::hex << std::showbase << std::uppercase;

   std::copy( data.begin( ), data.end( ), std::ostream_iterator<unsigned int>(result, " "));

   return result.str( );
}
