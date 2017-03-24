/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#include <srslib_test/utils/MemoryWatch.hpp>


namespace srs {
namespace test {

MemoryWatch::MemoryWatch()
{
	startStats_ = mallinfo();
	updateStats();
}

MemoryWatch::~MemoryWatch()
{}

long MemoryWatch::getMemoryUsage()
{
	updateStats();
	return memoryUsed_;
}

bool MemoryWatch::isZero()
{
	updateStats();
	return memoryUsed_ == 0;
}

void MemoryWatch::updateStats()
{
	struct mallinfo currentStats = mallinfo();
	memoryUsed_ = (currentStats.uordblks - startStats_.uordblks) +
		(currentStats.hblkhd - startStats_.hblkhd);
}

} // namespace test
} // namespace srs
