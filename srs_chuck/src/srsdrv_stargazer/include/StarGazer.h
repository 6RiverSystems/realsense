/*
* StarGazerMessageProcessor.cpp
*
*  Created on: May 4, 2016
*      Author: cacioppo
*/


#ifndef STARGAZER_H_
#define STARGAZER_H_
#include <stdint.h>
#include <vector>
#include <functional>
#include <chrono>
#include <map>
#include <queue>
#include "StarGazerMessageProcessor.h"


namespace srs {

	class StarGazer {
	private:

		bool							m_bControllerFault;

		bool							m_bStargazerStarted;

		StarGazerMessageProcessor		m_messageProcessor;

		void ReadCallback(std::string type, std::string param);

		void OdometryCallback(int tagID, float x, float y, float z, float angle);


	public:

		StarGazer(const char *comPort);

		virtual ~StarGazer();

		void SetConnected(bool bIsConnected);

		void Configure();

		void AutoCalculateHeight();

		void Start();

		void Stop();

		void PumpMessageProcessor();

	private:

		// Message Processing

		void OnVersion(std::vector<std::string> vecParams);

	};

} /* namespace srs */

#endif /* STARGAZER_H_ */

