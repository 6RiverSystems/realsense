/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef MEMORYWATCH_HPP_
#define MEMORYWATCH_HPP_

#include <malloc.h>
#include <unistd.h>

namespace srs {
namespace test {

class MemoryWatch
{
public:
    MemoryWatch()
    {
        startStats_ = mallinfo();
        updateStats();
    }

    ~MemoryWatch()
    {}

    long getMemoryUsage()
    {
        updateStats();
        return memoryUsed_;
    }

    bool isZero()
    {
        updateStats();
        return memoryUsed_ == 0;
    }

private:

    void updateStats()
    {
        struct mallinfo currentStats = mallinfo();
        memoryUsed_ = (currentStats.uordblks - startStats_.uordblks) +
            (currentStats.hblkhd - startStats_.hblkhd);
    }

    long memoryUsed_;
    struct mallinfo startStats_;
};

} // namespace test
} // namespace srs

#endif // MEMORYWATCH_HPP_
