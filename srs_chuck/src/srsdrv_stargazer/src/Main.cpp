#include <stdio.h>
#include <iostream>
#include <string>
#include <thread>
#include "StarGazer.h"


using namespace srs;

void main() {

	

//	std::string testStr2("~^I52|-179.91|-18.53|-31.14|337.44");
	std::string testStr2("~$Version|2.1601.08");

//	std::regex testRegex2("~\\^I[0-9]*\\|(-?[0-9]*\\.[0-9]*)\\|(-?[0-9]*\\.[0-9]*)\\|(-?[0-9]*\\.[0-9]*)\\|(-?[0-9]*\\.[0-9]*)");
	std::regex testRegex2("~\\$([^\\|]*)(?:\\|([^\\`]*))?");

	std::smatch testMatch2;

	if (std::cout << "Match2 ? " << testStr2 << " " << std::regex_match(testStr2, testMatch2, testRegex2) << std::endl)
	{

		for (unsigned int i = 0; i < testMatch2.size(); i++) {
			std::cout << i << " --> " << testMatch2[i] << std::endl;
		}
	}

	StarGazer starGazer("COM2");
	starGazer.Configure();
//	starGazer.AutoCalculateHeight();
	starGazer.Start();
	while (1)
	{
		starGazer.PumpMessageProcessor();
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}



}