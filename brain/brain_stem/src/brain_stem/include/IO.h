/*
 * IO.h
 *
 *  Created on: Apr 4, 2016
 *      Author: dan
 */
#include <functional>

#ifndef SRC_IO_H_
#define SRC_IO_H_

namespace srs {

class IO {
public:

	virtual ~IO( ) { };

	/**
	    Opens the serial port

	    @pszName - serial port name (e.g. /dev/malg, /dev/ttyS0, etc.)
	    @readCallback - callback when data is read from the port (called from io thread)
	*/
	virtual void Open( const char* pszName, std::function<void(std::vector<char>)> readCallback ) = 0;

	/**
	    Checks to see if the serial port is open

	    @return true if open, false otherwise
	*/
	virtual bool IsOpen( ) const = 0;

	/**
	    Closes a serial port

	    @return true if open, false otherwise
	*/
	virtual void Close( ) = 0;

	/**
	    Writes data to a serial port

	    @buffer - data to send
	*/
	virtual void Write( const std::vector<char>& buffer ) = 0;

};

} /* namespace srs */

#endif /* SRC_IO_H_ */
