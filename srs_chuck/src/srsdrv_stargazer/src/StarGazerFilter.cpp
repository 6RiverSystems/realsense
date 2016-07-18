#include <math.h>
#include <stdio.h>
#include "StarGazerFilter.h"
#include <ros/ros.h>

namespace srs {

	StarGazerFilter::StarGazerFilter(const double velocityLimitMpS,
		const double accelerationLimitMpS2,
		const double jerLimitMpS3) :
		m_lastRawX(0.0), m_lastRawY(0.0), m_lastRawZ(0.0),
		m_lastX(0.0), m_lastY(0.0),
		m_lastXvel(0.0), m_lastYvel(0.0),
		m_lastXacc(0.0), m_lastYacc(0.0),
		m_tagCounts(0), m_lastTagID(-1),
		m_velocityLimitCMpS(velocityLimitMpS * 100),
		m_accelerationLimitCMpS2(accelerationLimitMpS2 * 100),
		m_jerkLimitCMpS3(jerLimitMpS3 * 100)
	{

	}

	StarGazerFilter::~StarGazerFilter()
	{
	}

	//  This function sharpens the data.  It is essentially undoing an IIR running average filter
	double StarGazerFilter::SharpenFilter(double currPoint, double lastPoint, double scale)
	{
		return (currPoint - (1 - scale)*lastPoint) / scale;
	}

	void StarGazerFilter::reportDeadZone()
	{
		m_lastTagID = -1;
	}


	bool StarGazerFilter::unfilterStargazerData(int tagID, float& x, float& y, float& z)
	{
		double dx = x, dy = y, dz = z;
		bool rc = unfilterStargazerData(tagID, dx, dy, dz);

		x = dx;
		y = dy;
		z = dz;

		return rc;
	}

	bool StarGazerFilter::unfilterStargazerData(int tagID, double& x, double& y, double& z)
	{
		bool returnValue = false;

		// If this is a new tag, we know nothing so reset out counter
		if (tagID != m_lastTagID)
		{
			m_lastTagID = tagID;
			m_tagCounts = 0;
		}
		m_tagCounts++;

		// The first thing we are going to do is sharpen the data
		// this is important for both our huristics to work better (errored points won't be averaged over time)
		// but also to remove any time delays that averaging will cause which is always bad for navigation.
		// If this is not the same as the last tagID, then we should just return false and store the last data

		double rawX = x;
		double rawY = y;
		double rawZ = z;

		// If we have at least one point of history, we can start processing our data
		if (m_tagCounts > 1)
		{
			x = SharpenFilter(rawX, m_lastRawX, m_scale);
			y = SharpenFilter(rawY, m_lastRawY, m_scale);
			z = SharpenFilter(rawZ, m_lastRawZ, m_scale);

			double Xvel = (x - m_lastX) / m_eventSpacingSec;
			double Xacc = (Xvel - m_lastXvel) / m_eventSpacingSec;
			double Yvel = (y - m_lastY) / m_eventSpacingSec;
			double Yacc = (Yvel - m_lastYvel) / m_eventSpacingSec;


			// To have all the data be valid we need at least five counts on the same tag
			// Dont do any further processing unless we have it - just return false
			if (m_tagCounts > 4)
			{
				double Xjerk = (Xacc - m_lastXacc) / m_eventSpacingSec;
				double Yjerk = (Yacc - m_lastYacc) / m_eventSpacingSec;

				double absAcc = sqrt((Xacc * Xacc) + (Yacc * Yacc));
				double absVel = sqrt((Xvel * Xvel) + (Yvel * Yvel));

				double absJerk = sqrt((Xjerk * Xjerk) + (Yjerk * Yjerk));

				// If we appears to be acceleating within limits, have a velocity within limits
				// and have a jerk within limits, then accept the data.
				if ((absVel < m_velocityLimitCMpS) && (absAcc < m_accelerationLimitCMpS2) && (absJerk < m_jerkLimitCMpS3))
				{
					returnValue = true;

					ROS_DEBUG_NAMED( "filter", "vel: %7.4f (%1.0f) acc: %6.2f (%1.0f) jerk: %7.1f (%1.0f)", absVel, m_velocityLimitCMpS, absAcc, m_accelerationLimitCMpS2, absJerk, m_jerkLimitCMpS3);
				}
				else 
				{
					ROS_DEBUG_NAMED( "filter", "vel: %7.4f (%1.0f) acc: %6.2f (%1.0f) jerk: %7.1f (%1.0f) <", absVel, m_velocityLimitCMpS, absAcc, m_accelerationLimitCMpS2, absJerk, m_jerkLimitCMpS3);
				}
			}

			// Save off our history so that we can use them next time we are called
			m_lastYacc = Yacc;
			m_lastXacc = Xacc;

			m_lastXvel = Xvel;
			m_lastYvel = Yvel;


			m_lastX = x;
			m_lastY = y;

		}

		m_lastRawX = rawX;
		m_lastRawY = rawY;
		m_lastRawZ = rawZ;

		return returnValue;
	}

}
