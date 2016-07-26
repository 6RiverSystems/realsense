/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef GRID2DUTILS_HPP_
#define GRID2DUTILS_HPP_

#include <iostream>
using namespace std;

namespace srs {
namespace test {

struct Grid2dUtils
{
    static void addRectangleCost(Grid2d& grid,
        int x1, int y1, int x2, int y2,
        int cost,
        const void* notes = nullptr)
    {
        for (int x = x1; x <= x2; ++x)
        {
            for (int y = y1; y <= y2; ++y)
            {
                // TODO: Fix this to match the new mechanism of notes
                grid.addValue(Grid2d::LocationType(x, y), cost, const_cast<void*>(notes));
            }
        }
    }

    static void addLocationCost(Grid2d& grid, int x, int y,
        int cost,
        const void* notes = nullptr)
    {
        grid.addValue(Grid2d::LocationType(x, y), cost, const_cast<void*>(notes));
    }
};

} // namespace test
} // namespace srs

#endif // GRID2DUTILS_HPP_
