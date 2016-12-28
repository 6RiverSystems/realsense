/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#include <ros/ros.h>

#include <BrainStem.hpp>

int main(int argc, char** argv)
{
	// Create the BrainStem node and run it
	srs::BrainStem brainStem("srsdrv_brainstem", argc, argv);

	brainStem.run();

	return 0;
}
