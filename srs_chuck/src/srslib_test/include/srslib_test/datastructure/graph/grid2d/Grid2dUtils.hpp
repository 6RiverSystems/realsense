/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <iostream>
using namespace std;

#include <srslib_framework/datastructure/graph/grid2d/Grid2d.hpp>

namespace srs {
namespace test {

struct Grid2dUtils
{
    static void addRectangleCost(Grid2d& grid,
        unsigned int xi, unsigned int yi, unsigned int xf, unsigned int yf,
        unsigned int cost)
    {
        int deltaX = BasicMath::sgn<int>(xf - xi);
        int deltaY = BasicMath::sgn<int>(yf - yi);

        unsigned int c = xi;
        unsigned int r = yi;
        do
        {
            do
            {
                grid.maxCost(Grid2d::Location(c, r), cost);
                c += deltaX;
            } while (c != xf);

            r += deltaY;
        } while (r != yf);
    }

    static void addStaticObstacle(Grid2d& grid,
        unsigned int xi, unsigned int yi, unsigned int xf, unsigned int yf,
        int sizeEnvelopeCells = 0, int costEnvelope = 0)
    {
        // First add the envelope, if specified
        if (sizeEnvelopeCells > 0 && costEnvelope > 0)
        {
            addRectangleCost(grid,
                xi - sizeEnvelopeCells, yi - sizeEnvelopeCells,
                xf + sizeEnvelopeCells, yf + sizeEnvelopeCells,
                costEnvelope);
        }

        // Add the static obstacle
        addRectangleCost(grid, xi, yi, xf, yf, Grid2d::COST_MAX);
    }

    static void addWeight(Grid2d& grid,
        unsigned int xi, unsigned int yi, unsigned int xf, unsigned int yf,
        int north, int east, int south, int west)
    {
        int deltaX = BasicMath::sgn<int>(xf - xi);
        int deltaY = BasicMath::sgn<int>(yf - yi);

        unsigned int c = xi;
        unsigned int r = yi;
        do
        {
            do
            {
                grid.setWeights(Grid2d::Location(c, r), north, east, south, west);
                c += deltaX;
            } while (c != xf);

            r += deltaY;
        } while (r != yf);
    }
};

} // namespace test
} // namespace srs
