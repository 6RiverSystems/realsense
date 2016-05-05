/*
 * SerialIO.h
 *
 *  Created on: Apr 4, 2016
 *      Author: dan
 */
#include <vector>
#include <memory>
#include <boost/asio.hpp>
#include <boost/assert.hpp>
#include <functional>
#include <stdexcept>
#include <thread>
#include <set>
#include <chrono>


#ifndef SRC_STARGAZERSERIALIO_H_
#define SRC_STARGAZERSERIALIO_H_

namespace srs {

class StarGazerSerialIO :
	private boost::noncopyable
{

public:
	StarGazerSerialIO( );

	virtual ~StarGazerSerialIO( );

	void Open( const char* pszName, std::function<void( std::vector<char> )> readCallback );

	bool IsOpen( ) const;

	void Close( );

	void Write( const std::vector<char>& buffer );

	void WriteSyncronous( const std::vector<char>& buffer );

private:

	void WriteInSerialThread( std::vector<char> buffer );

	void StartAsyncRead( );

	void OnWriteComplete( const boost::system::error_code& error, std::size_t size );

	void OnReadComplete( const boost::system::error_code& error, std::size_t size );

private:

    std::shared_ptr<std::thread>			m_Thread;

	boost::asio::io_service					m_IOService;

	boost::asio::io_service::work			m_oWork;

    boost::asio::serial_port				m_SerialPort;

	std::chrono::microseconds				m_interByteDelay;

    bool									m_bIsWriting;

    std::vector<char>						m_ReadBuffer;

    std::vector<char>						m_writeData;

    std::vector<char>						m_readData;

    std::function<void(std::vector<char>)>	m_readCallback;
};

} /* namespace srs */

#endif /* SRC_STARGAZERSERIALIO_H_ */
