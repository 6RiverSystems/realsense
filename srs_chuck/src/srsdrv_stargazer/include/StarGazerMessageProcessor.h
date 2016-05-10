/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#ifndef STARGAZER_MESSAGE_PROCESSOR_H_
#define STARGAZER_MESSAGE_PROCESSOR_H_

#include <stdint.h>
#include <vector>
#include <functional>
#include <chrono>
#include <map>
#include <queue>
#include <regex>
#include "StarGazerMessage.h"
#include "StarGazerSerialIO.h"

namespace srs {

class StarGazerSerialIO;

typedef std::function<void(int, float, float, float, float)> OdometryCallbackFn;

typedef std::function<void(std::string, std::string)> ReadCallbackFn;

class StarGazerMessageProcessor {

	struct QueuedMessage {
		std::string sTxMsg;
		std::string sRxMsg;
		float       timeoutSec;
	};

private:

	std::shared_ptr<IO>									m_pSerialIO;

	std::chrono::high_resolution_clock::time_point		m_lastTxTime;

	std::string											m_lastTxMessage;

	std::chrono::high_resolution_clock					m_highrezclk;

	std::queue<QueuedMessage>							m_txMessageQueue;

	std::string											m_lastAck;

	boost::regex										m_odometryRegex;

	boost::regex										m_messageRegex;

	ReadCallbackFn										m_readCallback;

	OdometryCallbackFn									m_odometryCallback;

private:

	void SendRawCommand( QueuedMessage msg );

	void BaseCommand( STAR_GAZER_MESSAGE_TYPES type, std::string cmd, std::string rxExpected, float timeoutSec );

	void BaseWriteCommand( std::string cmd );

	void BaseWriteCommand( std::string cmd, std::string arg1 );

	void BaseWriteCommand( std::string cmd, int arg1 );

	void BaseReadCommand( std::string cmd );

public:

	StarGazerMessageProcessor( std::shared_ptr<IO> pIO );

	virtual ~StarGazerMessageProcessor( );

	void ProcessStarGazerMessage( std::vector<char> msgBuffer );

	void SetOdometryCallback( OdometryCallbackFn odometryCallback );

	void SetReadCallback( ReadCallbackFn readCallback );

	void HardReset( );

	void CalcStop( );

	void CalcStart( );

	void SetEnd( );

	void SetMarkType( STAR_GAZER_LANDMARK_TYPES type );

	void HeightCalc( );

	void HightFix( bool fixHeight );

	void SetMarkHeight( int height_mm );

	void PumpMessageProcessor( );

	void GetVersion( );

private:

// Helper Methods

	void WriteToSerialPort( char* pszData, std::size_t dwSize );

};

} /* namespace srs */

#endif /* STARGAZER_MESSAGE_PROCESSOR_H_ */
