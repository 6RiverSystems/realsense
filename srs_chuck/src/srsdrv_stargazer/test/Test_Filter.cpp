#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include "StarGazerFilter.h"

namespace srs {
	float generateRandomFloat(float min, float max) {
		float random = ((float)rand()) / (float)RAND_MAX;
		float diff = max - min;
		float r = random * diff;
		return min + r;
	}

	double runningAverage(double x, double lastAvg, double scale)
	{
		return scale * x + (1 - scale) * lastAvg;
	}

	bool TestSharpenFilter()
	{
		StarGazerFilter filter(1.0, 1.0, 1.0);
		float x, lastAvg;
		const double scale = filter.m_scale;
		// Set the seed to a fixed value so that the test always has the same data
		srand(100);

		lastAvg = generateRandomFloat(-200.0, 200.0);
		for (int i = 0; i < 1000; i++) {
			x = generateRandomFloat(-200.0, 200.0);
			float recoveredX = filter.SharpenFilter(runningAverage(x, lastAvg, scale) , lastAvg, scale);

			if (abs(recoveredX - x) > 0.001) {
				return false;
			}
		}
		return true;

	}

	bool TestMultipleTags() 
	{
		StarGazerFilter filter(2.0, 1.0, 0.1);
		float x = -100.0;
		float y = 100.0;
		float z = 100.0;

		for (int tagID = 1; tagID < 14; tagID++)
		{
			// first four points should be returned false, the rest should be true
			for (int i = 0; i < 4; i++)
			{
				float tmpX = x;
				float tmpY = y;
				float tmpZ = z;
				bool keep = filter.unfilterStargazerData(tagID, tmpX, tmpY, tmpZ);
				if ((i < 4) && (keep))
				{
					return false;
				}
				else if ((i > 4) && (!keep))
				{
					return false;
				}
				x += 10.0;
			}
		}
		return true;
	}

	bool TestDeadZone() 
	{
		StarGazerFilter filter(2.0, 1.0, 0.1);
		float x = -100.0;
		float y = 100.0;
		float z = 100.0;
		int tagID = 1;

		// first four points should be returned false, the rest should be true
		for (int i = 0; i < 14; i++)
		{
			float tmpX = x;
			float tmpY = y;
			float tmpZ = z;
			bool keep = filter.unfilterStargazerData(tagID, tmpX, tmpY, tmpZ);
			if ((i < 4) && (keep)) 
			{
				return false;
			}
			else if ((i > 4) && (!keep))
			{
				return false;
			}
			x += 10.0;
		}

		// If we send a dead zone command, we should reset back to needing four more points
		filter.reportDeadZone();

		// first four points should be returned false, the rest should be true
		for (int i = 0; i < 14; i++)
		{
			float tmpX = x;
			float tmpY = y;
			float tmpZ = z;

			bool keep = filter.unfilterStargazerData(tagID, tmpX, tmpY, tmpZ);
			if ((i < 4) && (keep))
			{
				return false;
			}
			else if ((i > 4) && (!keep))
			{
				return false;
			}
			x += 10;
		}

		return true;
	}

	bool TestVelocityLimit()
	{
		const float MaxVelocityCMpS = 200.0;
		StarGazerFilter filter(MaxVelocityCMpS/100, 1.0, 10.0);
		float x = -100.0;
		float y = 100.0;
		float z = 100.0;
		int tagID = 1;
		float scale = filter.m_scale;
		float timeSpacingSec = filter.m_eventSpacingSec;
		float lastTmpX = x;

		// first four points should be returned false, the rest should be true
		for (int i = 0; i < 14; i++)
		{
			float tmpX = runningAverage(x, lastTmpX, scale);
			lastTmpX = tmpX;
			float tmpY = y;
			float tmpZ = z;
			bool keep = filter.unfilterStargazerData(tagID, tmpX, tmpY, tmpZ);
			if ((i < 4) && (keep))
			{
				return false;
			}
			else if ((i > 4) && (!keep))
			{
				return false;
			}
			x += MaxVelocityCMpS * 0.99 * timeSpacingSec;
		}

		// add a little so that the velcity is slightly violated
		x += MaxVelocityCMpS * 0.02 * timeSpacingSec;
		// first four points should be returned false, the rest should be true
		for (int i = 0; i < 4; i++)
		{
			float tmpX = runningAverage(x, lastTmpX, scale);
			lastTmpX = tmpX;
			float tmpY = y;
			float tmpZ = z;
			bool keep = filter.unfilterStargazerData(tagID, tmpX, tmpY, tmpZ);
			if ((i < 1) && (keep))
			{
				return false;
			}
			else if ((i > 1) && (!keep))
			{
				return false;
			}
			x += MaxVelocityCMpS * 0.99 * timeSpacingSec;
		}
		return true;
	}

	bool TestDiagnalVelocity()
	{
		const float MaxVelocityCMpS = 200.0;
		StarGazerFilter filter(MaxVelocityCMpS / 100.0, 1.0, 30000.0);
		float x = -100;
		float y = 100;
		float z = 100;
		int tagID = 1;
		float scale = filter.m_scale;
		float timeSpacingSec = filter.m_eventSpacingSec;
		float lastTmpX = x;
		float lastTmpY = y;


		// first four points should be returned false, the rest should be true
		for (int i = 0; i < 14; i++)
		{
			float tmpX = runningAverage(x, lastTmpX, scale);
			lastTmpX = tmpX;
			float tmpY = runningAverage(y, lastTmpY, scale);
			lastTmpY = tmpY;
			float tmpZ = z;
			bool keep = filter.unfilterStargazerData(tagID, tmpX, tmpY, tmpZ);
			if ((i < 4) && (keep))
			{
				return false;
			}
			else if ((i > 4) && (!keep))
			{
				return false;
			}
			x += MaxVelocityCMpS * 0.99 * timeSpacingSec / sqrt(2.0);
			y += MaxVelocityCMpS * 0.99 * timeSpacingSec / sqrt(2.0);
		}

		// add a little so that the velcity is slightly violated
		x += MaxVelocityCMpS * 0.02 * timeSpacingSec / sqrt(2.0);
		y += MaxVelocityCMpS * 0.02 * timeSpacingSec / sqrt(2.0);

		// first four points should be returned false, the rest should be true
		for (int i = 0; i < 4; i++)
		{
			float tmpX = runningAverage(x, lastTmpX, scale);
			lastTmpX = tmpX;
			float tmpY = runningAverage(y, lastTmpY, scale);
			lastTmpY = tmpY;
			float tmpZ = z;
			bool keep = filter.unfilterStargazerData(tagID, tmpX, tmpY, tmpZ);
			if ((i < 1) && (keep))
			{
				return false;
			}
			else if ((i > 1) && (!keep))
			{
				return false;
			}
			x += MaxVelocityCMpS * 0.99 * timeSpacingSec / sqrt(2.0);
			y += MaxVelocityCMpS * 0.99 * timeSpacingSec / sqrt(2.0);
		}

		return true;
	}

	bool TestAccelerationLimit()
	{
		const float MaxVelocityCMpS = 2000.0;
		const float MaxAccelereationCMpS = 100.0;
		StarGazerFilter filter(MaxVelocityCMpS / 100.0, MaxAccelereationCMpS / 100.0, 3000.0);
		float x = -100;
		float y = 100;
		float z = 100;
		int tagID = 1;
		float scale = filter.m_scale;
		float timeSpacingSec = filter.m_eventSpacingSec;
		float lastTmpX = x;
		float lastTmpY = y;
		float currVelocity = -MaxVelocityCMpS*0.9;



		// first four points should be returned false, the rest should be true
		for (int i = 0; i < 14; i++)
		{
			float tmpX = runningAverage(x, lastTmpX, scale);
			lastTmpX = tmpX;
			float tmpY = runningAverage(y, lastTmpY, scale);
			lastTmpY = tmpY;
			float tmpZ = z;
			bool keep = filter.unfilterStargazerData(tagID, tmpX, tmpY, tmpZ);
			if ((i < 4) && (keep))
			{
				return false;
			}
			else if ((i > 4) && (!keep))
			{
				return false;
			}
			currVelocity += MaxAccelereationCMpS * 0.99 * timeSpacingSec;
			x += currVelocity * timeSpacingSec / sqrt(2.0);
			y += currVelocity * timeSpacingSec / sqrt(2.0);
		}

		// add a little so that the acceleration is slightly violated
		x -= currVelocity *  timeSpacingSec / sqrt(2.0);
		y -= currVelocity *  timeSpacingSec / sqrt(2.0);

		currVelocity += MaxAccelereationCMpS * 0.02 * timeSpacingSec;

		x += currVelocity *  timeSpacingSec / sqrt(2.0);
		y += currVelocity *  timeSpacingSec / sqrt(2.0);

		// first four points should be returned false, the rest should be true
		for (int i = 0; i < 4; i++)
		{
			float tmpX = runningAverage(x, lastTmpX, scale);
			lastTmpX = tmpX;
			float tmpY = runningAverage(y, lastTmpY, scale);
			lastTmpY = tmpY;
			float tmpZ = z;
			bool keep = filter.unfilterStargazerData(tagID, tmpX, tmpY, tmpZ);
			if ((i < 1) && (keep))
			{
				return false;
			}
			else if ((i > 1) && (!keep))
			{
				return false;
			}
			currVelocity += MaxAccelereationCMpS * 0.99 * timeSpacingSec;
			x += currVelocity * timeSpacingSec / sqrt(2.0);
			y += currVelocity * timeSpacingSec / sqrt(2.0);
		}

		return true;
	}

	bool TestJerkLimit()
	{
		const float MaxVelocityCMpS = 20000.0;
		const float MaxAccelereationCMpS = 1000.0;
		const float MaxJerkCMpS = 100.0;
		StarGazerFilter filter(MaxVelocityCMpS / 100.0, MaxAccelereationCMpS / 100.0, MaxJerkCMpS/ 100.0);
		double x = -100;
		double y = 100;
		double z = 100;
		int tagID = 1;
		double scale = filter.m_scale;
		double timeSpacingSec = filter.m_eventSpacingSec;
		double lastTmpX = x;
		double lastTmpY = y;
		double currVelocity = -MaxVelocityCMpS*0.9;
		double currAcceleration = -MaxAccelereationCMpS*0.9;



		// first four points should be returned false, the rest should be true
		for (int i = 0; i < 14; i++)
		{
			double tmpX = runningAverage(x, lastTmpX, scale);
			lastTmpX = tmpX;
			double tmpY = runningAverage(y, lastTmpY, scale);
			lastTmpY = tmpY;
			double tmpZ = z;
			printf("(%f,%f)\n", x, y);
			bool keep = filter.unfilterStargazerData(tagID, tmpX, tmpY, tmpZ);
			if ((i < 4) && (keep))
			{
				return false;
			}
			else if ((i > 4) && (!keep))
			{
				return false;
			}
			currAcceleration += MaxJerkCMpS * 0.98 * timeSpacingSec;
			currVelocity += currAcceleration * timeSpacingSec;
			x += currVelocity * timeSpacingSec / sqrt(2.0);
			y += currVelocity * timeSpacingSec / sqrt(2.0);
		}

		// add a little so that the jerk is slightly violated
		x -= currVelocity *  timeSpacingSec / sqrt(2.0);
		y -= currVelocity *  timeSpacingSec / sqrt(2.0);
		currVelocity -= currAcceleration * timeSpacingSec;

		currAcceleration += MaxJerkCMpS * 0.04 * timeSpacingSec;
		currVelocity += currAcceleration * timeSpacingSec;
		x += currVelocity *  timeSpacingSec / sqrt(2.0);
		y += currVelocity *  timeSpacingSec / sqrt(2.0);

		// first four points should be returned false, the rest should be true
		for (int i = 0; i < 4; i++)
		{
			double tmpX = runningAverage(x, lastTmpX, scale);
			lastTmpX = tmpX;
			double tmpY = runningAverage(y, lastTmpY, scale);
			lastTmpY = tmpY;
			double tmpZ = z;
			bool keep = filter.unfilterStargazerData(tagID, tmpX, tmpY, tmpZ);
			if ((i < 1) && (keep))
			{
				return false;
			}
			else if ((i > 1) && (!keep))
			{
				return false;
			}
			currAcceleration += MaxJerkCMpS *0.98 * timeSpacingSec;
			currVelocity += currAcceleration * timeSpacingSec;
			x += currVelocity * timeSpacingSec / sqrt(2.0);
			y += currVelocity * timeSpacingSec / sqrt(2.0);
		}

		return true;
	}

	bool TestRealData()
	{
		const int setSize = 19;
		float xSet[setSize] = { -160.250000, -160.440002, -160.440002, -160.380005, -160.130005, -159.949997, -160.020004, -160.070007, -160.100006, -160.130005, -160.149994, -159.970001, -160.029999, -160.070007, -160.110001, -160.130005, -160.149994, -160.160004, -159.979996 };
		float ySet[setSize] = { -112.500000, -112.480003, -112.570000, -112.580002, -112.860001, -113.059998, -112.940002, -112.849998, -112.779999, -112.730003, -112.699997, -112.940002, -112.849998, -112.779999, -112.730003, -112.699997, -112.669998, -112.650002, -112.910004 };
		float zSet[setSize] = { 502.959991, 502.959991, 502.200012, 502.200012, 502.200012, 502.200012, 502.200012, 502.200012, 502.200012, 502.200012, 502.200012, 502.200012, 502.200012, 502.200012, 502.200012, 502.200012, 502.200012, 502.200012, 502.200012 };
		int tagID = 120;
		StarGazerFilter filter(2.6,5.0,80);


		// first four points should be returned false, the rest should be true
		for (int i = 0; i < setSize; i++)
		{

			//printf("(%f,%f)\n", xSet[i], ySet[i]);
			bool keep = filter.unfilterStargazerData(tagID, xSet[i], ySet[i], zSet[i]);
			if ((i < 4) && (keep))
			{
				return false;
			}
			else if ((i > 4) && (!keep))
			{
				return false;
			}
		}

		return true;
	}

	void runTests()
	{
		if (TestSharpenFilter())
		{
			printf("TestSharpenFilter passed\n");
		}
		else
		{
			printf("TestSharpenFilter FAILTED!\n");
		}

		if (TestMultipleTags())
		{
			printf("TestMultipleTags passed\n");
		}
		else
		{
			printf("TestMultipleTags FAILTED!\n");
		}

		if (TestDeadZone())
		{
			printf("TestDeadZone passed\n");
		}
		else
		{
			printf("TestDeadZone FAILTED!\n");
		}

		if (TestVelocityLimit())
		{
			printf("TestVelocityLimit passed\n");
		}
		else
		{
			printf("TestVelocityLimit FAILTED!\n");
		}
		
		if (TestDiagnalVelocity())
		{
			printf("TestDiagnalVelocity passed\n");
		}
		else
		{
			printf("TestDiagnalVelocity FAILTED!\n");
		}

		if (TestAccelerationLimit())
		{
			printf("TestAccelerationLimit passed\n");
		}
		else
		{
			printf("TestAccelerationLimit FAILTED!\n");
		}

		if (TestJerkLimit())
		{
			printf("TestJerkLimit passed\n");
		}
		else
		{
			printf("TestJerkLimit FAILTED!\n");
		}


		if (TestRealData())
		{
			printf("TestRealData passed\n");
		}
		else
		{
			printf("TestRealData FAILTED!\n");
		}
	}
}
