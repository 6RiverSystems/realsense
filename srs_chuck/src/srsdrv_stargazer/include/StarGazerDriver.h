/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#ifndef STARGAZER_DRIVER_H_
#define STARGAZER_DRIVER_H_

#include <stdint.h>
#include <vector>
#include <functional>
#include <chrono>
#include <map>
#include <queue>
#include "StarGazerMessageProcessor.h"

namespace srs
{

class StarGazerDriver
{

private:

	bool						m_bStarted;

	StarGazerMessageProcessor	m_messageProcessor;

public:

	StarGazerDriver( const std::string& pszSerialPort );

	virtual ~StarGazerDriver( );

	void SetOdometryCallback( OdometryCallbackFn callback );

	void SetConnected( bool bIsConnected );

	void HardReset( );

	void Configure( );

	void AutoCalculateHeight( );

	void Start( );

	void Stop( );

	void PumpMessageProcessor( );

	void ReadCallback( std::string strType, std::string strValue );

private:

// Message Processing

	void OnVersion( std::vector<std::string> vecParams );

};

} /* namespace srs */

#endif /* STARGAZER_DRIVER_H_ */

