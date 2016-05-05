/*
 * StarGazerMessageProcessor.h
 *
 *  Created on: Apr 27, 2016
 *      Author: cacioppo
 */

#ifndef STARGAZERMESSAGEPROCESSOR_H_
#define STARGAZERMESSAGEPROCESSOR_H_

#include <stdint.h>
#include <vector>
#include <functional>
#include <chrono>
#include <map>
#include <queue>
#include <regex>
#include "StarGazerMessage.h"

namespace srs {

class StarGazerSerialIO;
class StarGazerMessageProcessor;


typedef std::function<void(int, float, float, float, float)> OdometryCallbackFn;

typedef std::function<void(std::string, std::string)> ReadCallbackFn;

class StarGazerMessageProcessor {

private:

	StarGazerSerialIO*									m_pIO;

	std::chrono::high_resolution_clock::time_point		m_lastTxTime;

	std::string											m_lastTxMessage;

	std::chrono::high_resolution_clock					m_highrezclk;

	std::queue<std::string>								m_txMessageQueue;

	std::string											m_lastAck;

	std::regex											m_odometryRegex;

	std::regex											m_messageRegex;

	ReadCallbackFn										m_readCallback;

	OdometryCallbackFn									m_odometryCallback;

private:

	void SendRawCommand( std::string fullCmd );

	void BaseCommand( STAR_GAZER_MESSAGE_TYPES type, std::string cmd );

	void BaseWriteCommand( std::string cmd );

	void BaseWriteCommand( std::string cmd, std::string arg1 );

	void BaseWriteCommand( std::string cmd, int arg1 );

	void BaseReadCommand( std::string cmd );

	void RxMsgCallback( std::vector<char> msgBuffer );

public:

	//StarGazerMessageProcessor(StarGazerSerialIO* pIO);
	StarGazerMessageProcessor( const char *comPort,
		std::function<void( std::string msg, std::string param )> readCallback,
		std::function<void( int tagID, float x, float y, float z, float angle )> odometryCallback );

	virtual ~StarGazerMessageProcessor( );

	void SetConnected( bool bIsConnected );

	void CalcStop( );

	void CalcStart( );

	void SetEnd( );

	void SetMarkType( STAR_GAZER_LANDMARK_TYPES type );

	void HeightCalc( );

	void SetMarkHeight( int height_mm );

	void PumpMessageProcessor( );

	void GetVersion( );

private:

// Helper Methods

	void WriteToSerialPort( char* pszData, std::size_t dwSize );

};

} /* namespace srs */

#endif /* STARGAZERMESSAGEPROCESSOR_H_ */
