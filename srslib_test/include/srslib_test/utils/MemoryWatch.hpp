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
    MemoryWatch();

    ~MemoryWatch();

    long getMemoryUsage();

    bool isZero();

private:

    void updateStats();

    long memoryUsed_;
    struct mallinfo startStats_;
};

} // namespace test
} // namespace srs

#endif // MEMORYWATCH_HPP_
