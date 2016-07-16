/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#include <ros/ros.h>

#include <srslib_framework/utils/Logging.hpp>
#include <srslib_framework/utils/CrashHandler.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <exception>      // std::set_terminate

#include <execinfo.h>
#include <signal.h>

#define UNW_LOCAL_ONLY
#include <libunwind.h>
#include <stdio.h>

namespace srs
{

std::string m_applicationPath;
std::string m_applicationName;

class CrashHandler
{
private:


public:

	CrashHandler( )
	{
		std::set_terminate( &CrashHandler::onUncaughtException );

		m_applicationPath = getExecutablePath( );

		boost::filesystem::path path( m_applicationPath );

		m_applicationName = path.filename( ).c_str( );

	    ROS_INFO_STREAM( "Initializing crash handler for: " << m_applicationName );

		struct sigaction sigact;

		sigact.sa_sigaction = errorHandler;
		sigemptyset( &sigact.sa_mask );
		sigact.sa_flags = SA_RESTART | SA_SIGINFO;

		if( sigaction( SIGSEGV, &sigact, (struct sigaction *) NULL ) != 0 )
		{
			ROS_ERROR( "Error setting signal handler for %d (%s)\n",
				SIGSEGV, strsignal( SIGSEGV ) );

			exit( EXIT_FAILURE );
		}
	}

private:

	static std::string exec( const char* command )
	{
		char buffer[128];
		std::string result;
		std::shared_ptr<FILE> pipe( popen( command, "r" ), pclose );
		if( pipe )
		{
			while( !feof( pipe.get( ) ) )
			{
				if( fgets( buffer, 128, pipe.get( ) ) != NULL )
				{
					 result += buffer;
				}
			}
		}

		return result;
	}

	std::string getExecutablePath( )
	{
		char exePath[PATH_MAX] = { '\0' };
		ssize_t len = ::readlink( "/proc/self/exe", exePath, sizeof(exePath) );
		if( len == -1 || len == sizeof(exePath) )
		{
			len = 0;
		}
		exePath[len] = '\0';

		return std::string( exePath );
	}

	static bool getFileAndLine( unw_word_t addr, std::string& function, std::string& file, uint32_t& lineNumber )
	{
		bool success = false;

		std::string addr2line = string_format( "/usr/bin/addr2line -C -e %s -f -i %lx", m_applicationPath.c_str( ), addr );

		std::string output = exec( addr2line.c_str( ) );

		if( output.size( ) )
		{
			if( output[0] != '?' )
			{
				std::vector<std::string> tokens;
				boost::split( tokens, output, boost::is_any_of( "\n" ) );

				if( tokens.size( ) >=2 )
				{
					function = tokens[0];
					std::string fileInfo = tokens[1];

					std::vector<std::string> fileTokens;
					boost::split( fileTokens, fileInfo, boost::is_any_of( ": " ) );

					if( fileTokens.size( ) >= 2 )
					{
						file = fileTokens[0];

						try
						{
							lineNumber = boost::lexical_cast<int32_t>( fileTokens[1] );

							success = true;
						}
						catch( const boost::bad_lexical_cast& e )
						{
							// Ignore (some output does not contain file line numbers)
						}
					}
				}
			}
		}

		return success;
	}

	static void errorHandler( int sig_num, siginfo_t * info, void * ucontext )
	{
	    ROS_ERROR( "%s crashed (signal: %d)", m_applicationName.c_str( ), sig_num );

		printStackTrace( );

		exit( EXIT_FAILURE );
	}

	static void onUncaughtException( )
	{
	    ROS_ERROR( "%s crashed (uncaught exception)", m_applicationName.c_str( ) );

		printStackTrace( );

		exit( EXIT_FAILURE );
 	}

	static void printStackTrace( )
	{
		char unwindMethod[] = "UNW_ARM_UNWIND_METHOD=4";
		putenv( unwindMethod );

		std::string name;
		unw_word_t ip, sp, offp;

		unw_context_t uc;
		unw_getcontext( &uc );

		unw_cursor_t cursor;
		unw_init_local( &cursor, &uc );

		uint32_t index = 0;

		std::string crashDump;

		while( unw_step( &cursor ) > 0 )
		{
			std::string function;
			std::string file;
			uint32_t lineNumber = 0;

			name.clear( );
			name.resize( 255 );

			if( unw_get_proc_name( &cursor, (char*)name.c_str( ), 256, &offp ) == 0 )
			{
				unw_get_reg( &cursor, UNW_REG_IP, &ip );
				unw_get_reg( &cursor, UNW_REG_SP, &sp );

				char* stringBuffer = nullptr;

				if( getFileAndLine( (long) ip, function, file, lineNumber ) )
				{
					crashDump += string_format( "%d => %s => %s (%d): %s\n", index, file.c_str( ), function.c_str( ),
						lineNumber, name.c_str( ) );
				}
				else
				{
					crashDump += string_format( "%d => Unknown File =>  %s\n", index, name.c_str( ) );
				}

				index++;
			}
		}

	    ROS_ERROR( "%s crashed: \n%s", m_applicationName.c_str( ), crashDump.c_str( ) );
	}

} g_crashHandler;

} // namespace srs
