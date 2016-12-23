/*
 * HidIO.h
 *
 *  Created on: Apr 4, 2016
 *      Author: dan
 */

#include <srslib_framework/io/IO.hpp>

#pragma once

namespace srs {

struct libusb_context;
struct libusb_device_handle;

class HidIO :
	public IO,
	private boost::noncopyable
{

public:

	HidIO( const char* pszName );

	virtual ~HidIO( );

	void Open( const char* pszName, ConnectionCallbackFn connectionCallback,
		ReadCallbackFn readCallback );

	bool IsOpen( ) const;

	void Close( );

// Write Methods

	void Write( const std::vector<char>& buffer );

private:

	std::string								m_strName;

    ReadCallbackFn							m_readCallback;

};

} /* namespace srs */
