/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#include <chrono>
#include <srslib_framework/io/SerialIO.hpp>

#ifndef STARGAZER_SERIAL_IO_H_
#define STARGAZER_SERIAL_IO_H_

namespace srs {

class StarGazerSerialIO :
	public SerialIO
{

public:
	StarGazerSerialIO( );

	virtual ~StarGazerSerialIO( );

private:

	std::chrono::microseconds				m_interByteDelay;

};

} /* namespace srs */

#endif /* STARGAZER_SERIAL_IO_H_ */
