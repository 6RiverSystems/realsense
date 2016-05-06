/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef COMPARE_HPP_
#define COMPARE_HPP_

#include <iostream>
using namespace std;

namespace srs {
namespace test {

struct Grid2dUtils
{
    static void addRectangleCost(Grid2d& grid, int x1, int y1, int x2, int y2, int cost)
    {
        for (int x = x1; x <= x2; ++x) {
            for (int y = y1; y <= y2; ++y) {
                grid.addValue(Grid2d::LocationType(x, y), ++cost);

                //cout << grid << endl;
            }
        }
    }
};

} // namespace test
} // namespace srs

#endif // COMPARE_HPP_
