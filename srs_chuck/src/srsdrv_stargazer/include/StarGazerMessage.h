/*
* StargazerMessage.h
*
*  Created on: Apr 29, 2016
*      Author: cacioppoc
*/
#include <stdint.h>
#include <limits>
#include <vector>
#include <boost/regex.hpp>

#ifndef _STARGAZER_MESSAGE_H_
#define _STARGAZER_MESSAGE_H_

namespace srs {

	const char STARGAZER_STX = '~';
	const char STARGAZER_RTX = '`';
	const char STARGAZER_SEPERATOR = '|';

	const float STARTGAZER_TIMEOUT = 0.25;

	enum class STAR_GAZER_MESSAGE_TYPES
	{
		READ = '@',
		WRITE = '#',
		RETURN_VALUE = '$',
		ACK = '!',
		MESSAGE = '*',
		POSE = '^',
		UNKNOWN
	};

	const char STARGAZER_LANDMARK_STIRNGS[][6] = { "HLD1S", "HLD1L", "HLD2S", "HLD2L", "HLD3S", "HLD3L" };

	enum class STAR_GAZER_LANDMARK_TYPES
	{
		HLD1S = 0,
		HLD1L = 1,
		HLD2S = 2,
		HLD2L = 3,
		HLD3S = 4,
		HLD3L = 5,
		UNKNOWN
	};
}

class StarGazerMessage
{
	StarGazerMessage(srs::STAR_GAZER_MESSAGE_TYPES type, std::vector<char> subType);

	virtual ~StarGazerMessage();

public:

	void appendParameter(std::vector<char> param);

	std::vector<char> getBuffer();
};

#endif //_STARGAZER_MESSAGE_H_
