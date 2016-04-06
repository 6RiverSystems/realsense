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
#include <thread>
#include <set>

#include "IO.h"

#ifndef SRC_SERIALIO_H_
#define SRC_SERIALIO_H_

namespace srs {

class SerialIO :
	public IO,
	private boost::noncopyable
{

public:
	SerialIO( bool bGenerateCRC = false, bool bIncludeLength = true,
		char cTerminating = '\n', char cEscape = '\\',
		std::set<char> vecCharsToEscape = std::set<char>( { '\\', '\n' } ) );

	virtual ~SerialIO( );

	void Open( const char* pszName, std::function<void(std::vector<char>)> readCallback );

	bool IsOpen( ) const;

	void Close( );

	void Write( const std::vector<char>& buffer );

private:

	void WriteInSerialThread( std::vector<char> buffer );

	void StartAsyncRead( );

	void OnWriteComplete( const boost::system::error_code& error, std::size_t size );

	void OnReadComplete( const boost::system::error_code& error, std::size_t size );

private:

	bool									m_bGenerateCRC;

	bool									m_bIncludeLength;

	char									m_cTerminating;

	char									m_cEscape;

	std::set<char>							m_setCharsToEscape;

    std::shared_ptr<std::thread>			m_Thread;

	boost::asio::io_service					m_IOService;

	boost::asio::io_service::work			m_oWork;

    boost::asio::serial_port				m_SerialPort;

    bool									m_bIsWriting;

    std::vector<char>						m_ReadBuffer;

    std::vector<char>						m_writeData;

    std::vector<char>						m_readData;

    std::function<void(std::vector<char>)>	m_readCallback;
};

} /* namespace srs */

#endif /* SRC_SERIALIO_H_ */
