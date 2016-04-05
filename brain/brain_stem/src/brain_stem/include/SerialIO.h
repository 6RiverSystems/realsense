/*
 * SerialIO.h
 *
 *  Created on: Apr 4, 2016
 *      Author: dan
 */
#include <boost/asio.hpp>
#include <boost/assert.hpp>
#include <functional>
#include <stdexcept>

#include "IO.h"

#ifndef SRC_SERIALIO_H_
#define SRC_SERIALIO_H_

namespace srs {

class SerialIO :
	public IO,
	private boost::noncopyable
{

public:
	SerialIO( );
	virtual ~SerialIO();

	void Open( const std::string& strName, std::function<void(std::vector<char>)> readCallback );

	bool IsOpen( ) const;

	void Close( );

	void Write( std::vector<char> buffer );

private:

	void StartAsyncRead( );

	void OnReadComplete( const boost::system::error_code& error, std::size_t size );

private:

	boost::asio::io_service					m_IOService;

    boost::asio::serial_port				m_SerialPort;

    boost::asio::streambuf					m_ReadBuffer;

    boost::asio::streambuf					m_WriteBuffer;

    std::vector<char>						m_readData;

    std::function<void(std::vector<char>)>	m_readCallback;
};

} /* namespace srs */

#endif /* SRC_SERIALIO_H_ */
