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

#include <srslib_framework/io/IO.hpp>

#ifndef SRC_SerialIO_H_
#define SRC_SerialIO_H_

namespace srs {

class SerialIO :
	public IO,
	private boost::noncopyable
{
	typedef std::shared_ptr<boost::asio::deadline_timer> ConnectionTimer;

	enum class READ_STATE
	{
		DEFAULT,
		IN_MESSAGE,
		IN_MESSAGE_ESCAPED
	};

public:
	SerialIO( );

	virtual ~SerialIO( );

	void Open( const char* pszName, std::function<void(std::vector<char>)> readCallback );

	bool IsOpen( ) const;

	void Close( );

// Serial IO Configuration

	void SetRetryTimeout( float fRetryTimeout );

	void EnableCRC( bool bGenerateCRC );

	void SetIncludeLength( bool bIncludeLength );

	void SetLeadingCharacter( char cLeading );

	void SetTerminatingCharacter( char cTerminating );

	void SetEscapeCharacter( char cEscape );

	void SetEscapeCharacters( std::set<char> vecCharsToEscape );

	void SetFirstByteDelay( std::chrono::microseconds firstByteDelay );

	void SetByteDelay( std::chrono::microseconds byteDelay );

// Write Methods

	void Write( const std::vector<char>& buffer );

	void WriteRaw( const std::vector<char>& buffer );

private:

	void WriteInSerialThread( std::vector<char> buffer, bool bIsRaw );

	void StartAsyncRead( );

	void OnWriteComplete( const boost::system::error_code& error, std::size_t size );

	void OnReadComplete( const boost::system::error_code& error, std::size_t size );

// Connection Retry Logic

	void OnCheckSerialPort( bool bInitialCheck, const boost::system::error_code& e =
		boost::system::error_code( ) );

private:

    std::shared_ptr<std::thread>			m_Thread;

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

	bool									m_bGenerateCRC;

	bool									m_bIncludeLength;

	bool									m_bHasLeading;

	char									m_cLeading;

	bool									m_bHasTerminating;

	char									m_cTerminating;

	bool									m_bHasEscape;

	char									m_cEscape;

	std::set<char>							m_setCharsToEscape;

	std::chrono::microseconds				m_firstByteDelay;

	std::chrono::microseconds				m_byteDelay;

	std::chrono::microseconds				m_interByteDelay;

};

} /* namespace srs */

#endif /* SRC_SerialIO_H_ */
