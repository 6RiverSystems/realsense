/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#include <gtest/gtest.h>
#include <ros/ros.h>

#include <vector>
using namespace std;

#include <srslib_framework/datastructure/graph/grid2d/Grid2d.hpp>
using namespace srs;

#include <srslib_test/datastructure/graph/grid2d/Grid2dUtils.hpp>

const Grid2d::Location P_1_1 = Grid2d::Location(1, 1);
const Grid2d::Location P_2_2 = Grid2d::Location(2, 2);
const Grid2d::Location P_3_3 = Grid2d::Location(3, 3);

TEST(Test_Graph2d, BasicSet)
{
    Grid2d grid(10);

    grid.setPayload(P_1_1, 10);
    grid.setPayload(P_3_3, 30);

    ASSERT_EQ(10, grid.getPayload(P_1_1)) <<
        "Location payload in " << P_1_1 << " is not as expected";
    ASSERT_EQ(30, grid.getPayload(P_3_3)) <<
        "Location payload in " << P_3_3 << " is not as expected";

    grid.setPayload(P_3_3, 15);

    ASSERT_EQ(10, grid.getPayload(P_1_1)) <<
        "Location payload in " << P_1_1 << " is not as expected";
    ASSERT_NE(30, grid.getPayload(P_3_3)) <<
        "Location payload in " << P_3_3 << " is not as expected";
    ASSERT_EQ(15, grid.getPayload(P_3_3)) <<
        "Location payload in " << P_3_3 << " is not as expected";
}

TEST(Test_Graph2d, GetPayload)
{
    Grid2d grid(10);

    grid.setPayload(P_1_1, 10);
    grid.setPayload(P_3_3, 30);

    ASSERT_EQ(10, grid.getPayload(P_1_1)) <<
        "Location payload in " << P_1_1 << " is not as expected";
    ASSERT_EQ(30, grid.getPayload(P_3_3)) <<
        "Location payload in " << P_3_3 << " is not as expected";
    ASSERT_EQ(srs::Grid2d::PAYLOAD_NO_INFORMATION, grid.getPayload(P_2_2)) <<
        "Location payload in " << P_2_2 << " is not as expected";
}

TEST(Test_Graph2d, BasicMax)
{
    Grid2d grid(10);

    grid.setPayload(P_1_1, 10);
    grid.setPayload(P_3_3, 30);

    ASSERT_EQ(10, grid.getPayload(P_1_1)) <<
        "Location payload in " << P_1_1 << " is not as expected";
    ASSERT_EQ(30, grid.getPayload(P_3_3)) <<
        "Location payload in " << P_3_3 << " is not as expected";

    grid.maxOnPayload(P_1_1, 5);
    grid.maxOnPayload(P_3_3, 50);
    grid.maxOnPayload(P_2_2, 50);

    ASSERT_EQ(10, grid.getPayload(P_1_1)) <<
        "Location payload in " << P_1_1 << " is not as expected";
    ASSERT_EQ(50, grid.getPayload(P_3_3)) <<
        "Location payload in " << P_3_3 << " is not as expected";
    ASSERT_EQ(50, grid.getPayload(P_2_2)) <<
        "Location payload in " << P_2_2 << " is not as expected";

    grid.maxOnPayload(P_2_2, srs::Grid2d::PAYLOAD_MAX);

    ASSERT_EQ(srs::Grid2d::PAYLOAD_MAX, grid.getPayload(P_2_2)) <<
        "Location payload in " << P_2_2 << " is not as expected";
}

TEST(Test_Graph2d, SetWithMax)
{
    Grid2d grid(10);

    grid.setPayload(P_1_1, 10);
    grid.setPayload(P_3_3, 30);

    ASSERT_EQ(10, grid.getPayload(P_1_1)) <<
        "Location payload in " << P_1_1 << " is not as expected";
    ASSERT_EQ(30, grid.getPayload(P_3_3)) <<
        "Location payload in " << P_3_3 << " is not as expected";

    grid.setPayload(P_3_3, srs::Grid2d::PAYLOAD_MAX);

    ASSERT_EQ(10, grid.getPayload(P_1_1)) <<
        "Location payload in " << P_1_1 << " is not as expected";
    ASSERT_EQ(srs::Grid2d::PAYLOAD_MAX, grid.getPayload(P_3_3)) <<
        "Location payload in " << P_3_3 << " is not as expected";
}

TEST(Test_Graph2d, AddWithMax)
{
    Grid2d grid(10);

    grid.setPayload(P_1_1, 10);
    grid.setPayload(P_3_3, 30);

    ASSERT_EQ(10, grid.getPayload(P_1_1)) <<
        "Location payload in " << P_1_1 << " is not as expected";
    ASSERT_EQ(30, grid.getPayload(P_3_3)) <<
        "Location payload in " << P_3_3 << " is not as expected";

    grid.maxOnPayload(P_3_3, srs::Grid2d::PAYLOAD_MAX);

    ASSERT_EQ(10, grid.getPayload(P_1_1)) <<
        "Location payload in " << P_1_1 << " is not as expected";
    ASSERT_EQ(srs::Grid2d::PAYLOAD_MAX, grid.getPayload(P_3_3)) <<
        "Location payload in " << P_3_3 << " is not as expected";
}

TEST(Test_Graph2d, BasicWeigths)
{
    Grid2d grid(10);

    grid.setPayload(P_1_1, 10);
    grid.setPayload(P_3_3, 30);
    grid.setWeights(P_3_3, 11, 22, 33, 44);

    ASSERT_EQ(Grid2d::WEIGHT_NO_INFORMATION,
        grid.getWeight(Grid2d::Position(P_1_1, Grid2d::ORIENTATION_NORTH))) <<
        "North weight in location " << P_1_1 << " is not as expected";

    ASSERT_EQ(11, grid.getWeight(Grid2d::Position(P_3_3, Grid2d::ORIENTATION_NORTH))) <<
        "North weight in location " << P_3_3 << " is not as expected";

    ASSERT_EQ(1, grid.getWeightCount()) <<
        "Unexpected number of weights";

    // if set weights of P_3_3 to WIGHT_NO_INFORMATION, the weightCount_ should become 0
    grid.setWeights(P_3_3, Grid2d::WEIGHT_NO_INFORMATION, Grid2d::WEIGHT_NO_INFORMATION,
        Grid2d::WEIGHT_NO_INFORMATION, Grid2d::WEIGHT_NO_INFORMATION);

    ASSERT_EQ(0, grid.getWeightCount()) <<
        "Unexpected number of weights";

    // if add weights back to P_3_3, weightCount_ should be 1
    grid.setWeights(P_3_3, Grid2d::WEIGHT_MIN, Grid2d::WEIGHT_MIN,
        Grid2d::WEIGHT_MIN, Grid2d::WEIGHT_MIN);

    ASSERT_EQ(1, grid.getWeightCount()) <<
        "Unexpected number of weights";

    grid.setWeights(P_1_1, 12, 23, 34, 45);

    ASSERT_EQ(12, grid.getWeight(Grid2d::Position(P_1_1, Grid2d::ORIENTATION_NORTH))) <<
        "North weight in location " << P_1_1 << " is not as expected";

    ASSERT_EQ(23, grid.getWeight(Grid2d::Position(P_1_1, Grid2d::ORIENTATION_EAST))) <<
        "East weight in location " << P_1_1 << " is not as expected";

    ASSERT_EQ(34, grid.getWeight(Grid2d::Position(P_1_1, Grid2d::ORIENTATION_SOUTH))) <<
        "South weight in location " << P_1_1 << " is not as expected";

    ASSERT_EQ(45, grid.getWeight(Grid2d::Position(P_1_1, Grid2d::ORIENTATION_WEST))) <<
        "West weight in location " << P_1_1 << " is not as expected";

    ASSERT_EQ(2, grid.getWeightCount()) <<
        "Unexpected number of weights";

    grid.setWeights(P_2_2, Grid2d::WEIGHT_NO_INFORMATION, Grid2d::WEIGHT_NO_INFORMATION,
        Grid2d::WEIGHT_NO_INFORMATION, Grid2d::WEIGHT_NO_INFORMATION);

    // weightCount_ should remain the same
    ASSERT_EQ(2, grid.getWeightCount()) <<
        "Unexpected number of weights";

    ASSERT_EQ(2, grid.getOccupiedCount()) <<
        "Unexpected number of occupied cells";
}

TEST(Test_Graph2d, BasicClear)
{
    Grid2d grid(10);

    grid.setPayload(P_3_3, 30);
    grid.setWeights(P_3_3, 11, 22, 33, 44);

    ASSERT_EQ(30, grid.getPayload(P_3_3)) <<
        "Location payload in " << P_3_3 << " is not as expected";

    ASSERT_EQ(11, grid.getWeight(Grid2d::Position(P_3_3, Grid2d::ORIENTATION_NORTH))) <<
        "North weight in location " << P_3_3 << " is not as expected";
    ASSERT_EQ(22, grid.getWeight(Grid2d::Position(P_3_3, Grid2d::ORIENTATION_EAST))) <<
        "East weight in location " << P_3_3 << " is not as expected";
    ASSERT_EQ(33, grid.getWeight(Grid2d::Position(P_3_3, Grid2d::ORIENTATION_SOUTH))) <<
        "South weight in location " << P_3_3 << " is not as expected";
    ASSERT_EQ(44, grid.getWeight(Grid2d::Position(P_3_3, Grid2d::ORIENTATION_WEST))) <<
        "West weight in location " << P_3_3 << " is not as expected";

    grid.clear(P_3_3);

    ASSERT_EQ(Grid2d::PAYLOAD_NO_INFORMATION, grid.getPayload(P_3_3)) <<
        "Location payload in " << P_3_3 << " is not as expected";

    ASSERT_EQ(Grid2d::PAYLOAD_NO_INFORMATION, grid.getWeight(Grid2d::Position(P_3_3, Grid2d::ORIENTATION_NORTH))) <<
        "North weight in location " << P_3_3 << " is not as expected";
    ASSERT_EQ(Grid2d::PAYLOAD_NO_INFORMATION, grid.getWeight(Grid2d::Position(P_3_3, Grid2d::ORIENTATION_EAST))) <<
        "East weight in location " << P_3_3 << " is not as expected";
    ASSERT_EQ(Grid2d::PAYLOAD_NO_INFORMATION, grid.getWeight(Grid2d::Position(P_3_3, Grid2d::ORIENTATION_SOUTH))) <<
        "South weight in location " << P_3_3 << " is not as expected";
    ASSERT_EQ(Grid2d::PAYLOAD_NO_INFORMATION, grid.getWeight(Grid2d::Position(P_3_3, Grid2d::ORIENTATION_WEST))) <<
        "West weight in location " << P_3_3 << " is not as expected";

    ASSERT_EQ(Grid2d::PAYLOAD_NO_INFORMATION, grid.getPayload(P_2_2)) <<
        "Location payload in " << P_2_2 << " is not as expected";

    ASSERT_EQ(Grid2d::PAYLOAD_NO_INFORMATION, grid.getWeight(Grid2d::Position(P_2_2, Grid2d::ORIENTATION_NORTH))) <<
        "North weight in location " << P_2_2 << " is not as expected";
    ASSERT_EQ(Grid2d::PAYLOAD_NO_INFORMATION, grid.getWeight(Grid2d::Position(P_2_2, Grid2d::ORIENTATION_EAST))) <<
        "East weight in location " << P_2_2 << " is not as expected";
    ASSERT_EQ(Grid2d::PAYLOAD_NO_INFORMATION, grid.getWeight(Grid2d::Position(P_3_3, Grid2d::ORIENTATION_SOUTH))) <<
        "South weight in location " << P_2_2 << " is not as expected";
    ASSERT_EQ(Grid2d::PAYLOAD_NO_INFORMATION, grid.getWeight(Grid2d::Position(P_3_3, Grid2d::ORIENTATION_WEST))) <<
        "West weight in location " << P_2_2 << " is not as expected";

    grid.clear(P_2_2);
}

TEST(Test_Grid2d, BasicBeginEnd)
{
    Grid2d grid(10);

    Grid2d::const_iterator beginIt = grid.begin();
    Grid2d::const_iterator endIt = grid.end();

    ASSERT_EQ(beginIt, endIt) << "The begin and end iterators do not match";

    grid.setPayload(P_3_3, 30);

    ASSERT_EQ(*grid.begin(), P_3_3) << "Unexpected iterator value";

    grid.setPayload(P_3_3, 30);
    grid.setPayload(P_3_3, 30);

    ASSERT_EQ(*grid.begin(), P_3_3) << "Unexpected iterator value";

    grid.setPayload(P_3_3, 30);
    grid.setPayload(P_2_2, 10);

    beginIt = grid.begin();
    ASSERT_EQ(*beginIt, P_2_2) << "Unexpected iterator value";

    ++beginIt;
    ASSERT_EQ(*beginIt, P_3_3) << "Unexpected iterator value";
}

TEST(Test_Grid2d, BasicForEach)
{
    Grid2d grid(10);

    Grid2d::const_iterator beginIt = grid.begin();
    Grid2d::const_iterator endIt = grid.end();

    ASSERT_EQ(beginIt, endIt) << "The begin and end iterators do not match";

    grid.setPayload(P_3_3, 30);
    grid.setPayload(P_3_3, 30);
    grid.setPayload(P_2_2, 10);

    int payloads = 0;
    for (Grid2d::Location it : grid)
    {
        switch (payloads)
        {
            case 0:
                ASSERT_EQ(P_2_2, it) << "Unexpected value";
                break;
            case 1:
                ASSERT_EQ(P_3_3, it) << "Unexpected value";
                break;
        }

        payloads++;
    }
}

TEST(Test_Grid2d, OccupiedCount)
{
    Grid2d grid(10);

    grid.setPayload(P_3_3, 30);

    ASSERT_EQ(1, grid.getOccupiedCount()) << "Unexpected occupied cells count";

    grid.setPayload(P_3_3, 30);
    grid.setPayload(P_3_3, 30);

    ASSERT_EQ(1, grid.getOccupiedCount()) << "Unexpected occupied cells count";

    grid.setPayload(P_3_3, 30);
    grid.setPayload(P_2_2, 10);

    ASSERT_EQ(2, grid.getOccupiedCount()) << "Unexpected occupied cells count";

    grid.setPayload(P_2_2, Grid2d::PAYLOAD_NO_INFORMATION);
    grid.setPayload(P_1_1, Grid2d::PAYLOAD_NO_INFORMATION);

    ASSERT_EQ(1, grid.getOccupiedCount()) << "Unexpected occupied cells count";

    grid.maxOnPayload(P_2_2, Grid2d::PAYLOAD_MIN);

    ASSERT_EQ(2, grid.getOccupiedCount()) << "Unexpected occupied cells count";
}

TEST(Test_Grid2d, EqualOperator)
{
    Grid2d correct(10);
    correct.setPayload(P_3_3, 3);
    correct.setPayload(P_1_1, 1);
    correct.setPayload(P_2_2, 2);
    correct.setWeights(P_2_2, 10, 20, 30, 40);

    Grid2d grid1(10);
    grid1.setPayload(P_3_3, 3);
    grid1.setPayload(P_1_1, 1);
    grid1.setPayload(P_2_2, 2);
    grid1.setWeights(P_2_2, 10, 20, 30, 40);

    ASSERT_EQ(grid1, grid1) << "The grid does not agree with itself";
    ASSERT_EQ(correct, grid1) << "The grids do not match";

    Grid2d grid2(10);
    grid2.setPayload(P_3_3, 30);

    ASSERT_NE(correct, grid2) << "The grids do match";

    Grid2d grid3(10);
    grid3.setPayload(P_3_3, 3);
    grid3.setPayload(P_1_1, 1);
    grid3.setPayload(P_2_2, 2);
    grid3.setWeights(P_2_2, 10, 0, 30, 40);

    ASSERT_NE(correct, grid3) << "The grids do match";
}
