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
#include <future>

#include <srslib_framework/io/IO.hpp>

#ifndef SRC_SerialIO_H_
#define SRC_SerialIO_H_

namespace srs {

class SerialIO :
	public IO,
	private boost::noncopyable
{
	typedef std::shared_ptr<boost::asio::deadline_timer> ConnectionTimer;

	typedef std::function<void(std::vector<char>)> ReadCallbackFn;

	enum class READ_STATE
	{
		DEFAULT,
		IN_MESSAGE,
		IN_MESSAGE_ESCAPED
	};

public:
	SerialIO( );

	virtual ~SerialIO( );

	void Open( const char* pszName, ReadCallbackFn readCallback );

	bool IsOpen( ) const;

	void Close( );

// Serial IO Configuration

	void SetRetryTimeout( float fRetryTimeout );

	void EnableCRC( bool bEnableCRC );

	void SetLeadingCharacter( char cLeading );

	void SetTerminatingCharacter( char cTerminating );

	void SetEscapeCharacter( char cEscape );

	void SetFirstByteDelay( std::chrono::microseconds firstByteDelay );

	void SetByteDelay( std::chrono::microseconds byteDelay );

// Write Methods

	void Write( const std::vector<char>& buffer );

private:

	void WriteInSerialThread( std::vector<char> writeBuffer );

	void StartAsyncRead( );

	void OnWriteComplete( const boost::system::error_code& error, std::size_t size );

	void OnReadComplete( const boost::system::error_code& error, std::size_t size );

// Connection Retry Logic

	bool OnCheckSerialPort( bool bInitialCheck, const boost::system::error_code& e =
		boost::system::error_code( ) );

private:

    std::shared_ptr<std::thread>			m_Thread;

    std::thread::id							m_serialThreadId;

	boost::asio::io_service					m_IOService;

	boost::asio::io_service::work			m_oWork;

	ConnectionTimer							m_oTimer;

    boost::asio::serial_port				m_SerialPort;

    bool									m_bIsSerialOpen;

	std::string								m_strSerialPort;

	float									m_fRetryTimeout;

    bool									m_bIsWriting;

    std::vector<char>						m_readBuffer;

    std::vector<char>						m_writeBuffer;

	READ_STATE								m_readState;

	uint8_t									m_cCRC = 0;

    std::vector<char>						m_readPartialData;

    std::function<void(std::vector<char>)>	m_readCallback;

	bool									m_bEnableCRC;

	bool									m_bHasLeading;

	char									m_cLeading;

	bool									m_bHasTerminating;

	char									m_cTerminating;

	bool									m_bHasEscape;

	char									m_cEscape;

	std::chrono::microseconds				m_firstByteDelay;

	std::chrono::microseconds				m_byteDelay;

	std::chrono::microseconds				m_interByteDelay;

};

} /* namespace srs */

#endif /* SRC_SerialIO_H_ */
