/*
* StarGazerFilter.cpp
*
*  Created on: May 29, 2016
*      Author: cacioppo
*
*
* This class is intended to clean up some of the bad data that
* the StarGazer returns.  There are two pieces of this data filtering, 
* a sharpening filter to un-do the built-in IIR running average filter
* and a bad point detector.
*/

#ifndef STAR_GAZER_FILTER_H
#define STAR_GAZER_FILTER_H

namespace srs 
{

	class StarGazerFilter;

	class StarGazerFilter
	{
	private:
		double m_lastRawX, m_lastRawY, m_lastRawZ;
		double m_lastX, m_lastY;
		double m_lastXvel, m_lastYvel;
		double m_lastXacc, m_lastYacc;
		int m_tagCounts, m_lastTagID;
		double m_velocityLimitCMpS;
		double m_accelerationLimitCMpS2;
		double m_jerkLimitCMpS3;

	public:		
		
		// The filter inside the stargazer appears to be y = 0.25*x + 0.75 * y-1
		const double m_scale = 0.25;

		// The Stargazer reports at 100ms intervals
		const double m_eventSpacingSec = 100.0 / 1000;

		StarGazerFilter(const double velocityLimitMpS = 2.60,
			const double accelerationLimitMpS2 = 0.80,
			const double jerLimitMpS3 = 1.0);

		virtual ~StarGazerFilter();

		// This should be called if the StarGazer reports an info that it is in a dead zone.
		// This is used to reset the filter and start from scratch
		void reportDeadZone();

		// This will filter the data to undo the running average IIR filter on the x, y and z data
		// This function will also do some huristics on the data to see if the data seems valid 
		// Sometimes the Stargazer will return invalid data - this only happens occasionally, usually near the very edge
		// of its visibility of the Tag or seeing sunlight.
		// The this data should only be considered valid if this funtion returns true. Otherwise, discard the data
		bool unfilterStargazerData(int tagID, float& x, float& y, float& z);
		bool unfilterStargazerData(int tagID, double& x, double& y, double& z);

		// This is a sharpen filter that is used internally.  This is public for unit testing
		static double SharpenFilter(double currPoint, double lastPoint, double scale);
	};


} /* namespace srs */

#endif /* STAR_GAZER_FILTER_H */
