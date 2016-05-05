/*
* StarGazerMessageProcessor.cpp
*
*  Created on: May 4, 2016
*      Author: cacioppo
*/
#include "StarGazer.h"
#include <iostream>

namespace srs {


StarGazer::StarGazer( const char *comPort ) :
	m_bControllerFault(false),
	m_bStargazerStarted(true),
	m_messageProcessor(comPort,
		std::bind(&StarGazer::ReadCallback, this, std::placeholders::_1, std::placeholders::_2),
		std::bind(&StarGazer::OdometryCallback, this, std::placeholders::_1, std::placeholders::_2,
			std::placeholders::_3, std::placeholders::_4, std::placeholders::_5)),
	m_odometryCallback( )
{

}

StarGazer::~StarGazer()
{

}

void StarGazer::SetOdometryCallback( OdometryCallbackFn callback )
{
	m_odometryCallback = callback;
}

void StarGazer::SetConnected(bool bIsConnected) 
{
	m_messageProcessor.SetConnected(bIsConnected);
}

void StarGazer::Configure()
{
	Stop();
	m_messageProcessor.GetVersion();
	m_messageProcessor.SetMarkType(STAR_GAZER_LANDMARK_TYPES::HLD3L);
	m_messageProcessor.SetEnd();
}

void StarGazer::AutoCalculateHeight()
{
	Stop();
	m_messageProcessor.HeightCalc();
}

void StarGazer::Start() {
	m_messageProcessor.CalcStart();
}

void StarGazer::Stop() {
	m_messageProcessor.CalcStop();
}

void StarGazer::PumpMessageProcessor() {
	m_messageProcessor.PumpMessageProcessor();
}

void StarGazer::ReadCallback(std::string type, std::string param) {
	if (type == "Version")
	{
		std::cout << "Version:" << param << std::endl;
	}
	else 
	{
		std::cout << "Unknown read: \"" << type << "\"" << std::endl;
	}
}

void StarGazer::OdometryCallback(int tagID, float x, float y, float z, float angle) {

	if( m_odometryCallback )
	{
		m_odometryCallback(tagID, x, y, z, angle);
	}
}

} /* namespace srs */
