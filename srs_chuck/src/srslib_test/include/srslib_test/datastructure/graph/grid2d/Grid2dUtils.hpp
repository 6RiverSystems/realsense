/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <iostream>
using namespace std;

#include <srslib_framework/datastructure/graph/grid2d/BaseGrid2d.hpp>
#include <srslib_framework/datastructure/graph/grid2d/WeightedGrid2d.hpp>
#include <srslib_framework/datastructure/Location.hpp>
#include <srslib_framework/localization/map/MapStack.hpp>
#include <srslib_framework/localization/map/MapAdapter.hpp>
#include <srslib_framework/localization/map/logical/LogicalMapFactory.hpp>
#include <srslib_framework/localization/map/occupancy/OccupancyMapFactory.hpp>

namespace srs {
namespace test {

struct Grid2dUtils
{
    static void addEmptySpace(SimpleGrid2d& grid)
    {
        addRectanglePayload(grid, 0, 0, grid.getWidth() - 1, grid.getHeight() - 1, 0);
    }

    static void addEmptySpace(SimpleGrid2d* grid)
    {
        addEmptySpace(*grid);
    }

    static void addObstacle(BaseGrid2d& grid,
        unsigned int xi, unsigned int yi, unsigned int xf, unsigned int yf,
        int sizeEnvelopeCells = 0, BaseGrid2d::BaseType costEnvelope = 0)
    {
        // First add the envelope, if specified
        if (sizeEnvelopeCells > 0 && costEnvelope > 0)
        {
            addRectanglePayload(grid,
                xi - sizeEnvelopeCells, yi - sizeEnvelopeCells,
                xf + sizeEnvelopeCells, yf + sizeEnvelopeCells,
                costEnvelope);
        }

        // Add the static obstacle
        addRectanglePayload(grid, xi, yi, xf, yf, grid.getMaxPayloadValue());
    }

    static void addObstacle(BaseGrid2d* grid,
        unsigned int xi, unsigned int yi, unsigned int xf, unsigned int yf,
        int sizeEnvelopeCells = 0, BaseGrid2d::BaseType costEnvelope = 0)
    {
        addObstacle(*grid, xi, yi, xf, yf, sizeEnvelopeCells, costEnvelope);
    }

    static void addRectanglePayload(BaseGrid2d& grid,
        unsigned int xi, unsigned int yi, unsigned int xf, unsigned int yf,
        BaseGrid2d::BaseType cost)
    {
        int deltaX = BasicMath::sgn<int>(xf - xi);
        int deltaY = BasicMath::sgn<int>(yf - yi);

        unsigned int c;
        unsigned int r = yi;
        do
        {
            c = xi;

            do
            {
                grid.payloadMax(Location(c, r), cost);
                c += deltaX;
            } while (c != (xf + deltaX));

            r += deltaY;
        } while (r != (yf + deltaY));
    }

    static void addRectanglePayload(BaseGrid2d* grid,
        unsigned int xi, unsigned int yi, unsigned int xf, unsigned int yf,
        BaseGrid2d::BaseType cost)
    {
        addRectanglePayload(*grid, xi, yi, xf, yf, cost);
    }

    static void addWeights(WeightedGrid2d& grid,
        unsigned int xi, unsigned int yi, unsigned int xf, unsigned int yf,
        WeightedGrid2d::BaseType north, WeightedGrid2d::BaseType northEast,
        WeightedGrid2d::BaseType east, WeightedGrid2d::BaseType southEast,
        WeightedGrid2d::BaseType south, WeightedGrid2d::BaseType southWest,
        WeightedGrid2d::BaseType west, WeightedGrid2d::BaseType northWest)
    {
        int deltaX = BasicMath::sgn<int>(xf - xi);
        int deltaY = BasicMath::sgn<int>(yf - yi);

        unsigned int c;
        unsigned int r = yi;
        do
        {
            c = xi;

            do
            {
                grid.setWeights(Location(c, r),
                    north, northEast,
                    east, southEast,
                    south, southWest,
                    west, northWest);
                c += deltaX;
            } while (c != (xf + deltaX));

            r += deltaY;
        } while (r != (yf + deltaY));
    }

    static void addWeights(WeightedGrid2d* grid,
        unsigned int xi, unsigned int yi, unsigned int xf, unsigned int yf,
        WeightedGrid2d::BaseType north, WeightedGrid2d::BaseType northEast,
        WeightedGrid2d::BaseType east, WeightedGrid2d::BaseType southEast,
        WeightedGrid2d::BaseType south, WeightedGrid2d::BaseType southWest,
        WeightedGrid2d::BaseType west, WeightedGrid2d::BaseType northWest)
    {
        addWeights(*grid, xi, yi, xf, yf,
            north, northEast,
            east, southEast,
            south, southWest,
            west, northWest);
    }

};

} // namespace test
} // namespace srs
