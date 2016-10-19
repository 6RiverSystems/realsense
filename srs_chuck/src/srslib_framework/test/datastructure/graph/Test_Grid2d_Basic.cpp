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
    ASSERT_EQ(srs::Grid2d::PAYLOAD_MIN, grid.getPayload(P_2_2)) <<
        "Location payload in " << P_3_3 << " is not as expected";
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

TEST(Test_Graph2d, BasicAggregate)
{
    Grid2d grid(10);

    grid.setAggregateSize(1, 1);

    grid.maxOnPayload(P_1_1, 10);
    grid.maxOnPayload(P_3_3, 30);

    ASSERT_EQ(10, grid.getPayload(P_1_1)) <<
        "Location payload in " << P_1_1 << " is not as expected";
    ASSERT_EQ(30, grid.getPayload(P_3_3)) <<
        "Location payload in " << P_3_3 << " is not as expected";

    ASSERT_EQ(10, grid.getAggregate(P_1_1)) <<
        "Location aggregate in " << P_1_1 << " is not as expected";
    ASSERT_EQ(30, grid.getAggregate(P_3_3)) <<
        "Location aggregate in " << P_3_3 << " is not as expected";
}

TEST(Test_Graph2d, AggregateChange)
{
    Grid2d grid(10);

    grid.setAggregateSize(1, 1);

    grid.maxOnPayload(P_1_1, 10);
    grid.maxOnPayload(P_3_3, 30);

    ASSERT_EQ(10, grid.getPayload(P_1_1)) <<
        "Location payload in " << P_1_1 << " is not as expected";
    ASSERT_EQ(30, grid.getPayload(P_3_3)) <<
        "Location payload in " << P_3_3 << " is not as expected";

    ASSERT_EQ(10, grid.getAggregate(P_1_1)) <<
        "Location aggregate in " << P_1_1 << " is not as expected";
    ASSERT_EQ(30, grid.getAggregate(P_3_3)) <<
        "Location aggregate in " << P_3_3 << " is not as expected";

    grid.setAggregateSize(2, 2);

    ASSERT_EQ(10, grid.getPayload(P_1_1)) <<
        "Location payload in " << P_1_1 << " is not as expected";
    ASSERT_EQ(30, grid.getPayload(P_3_3)) <<
        "Location payload in " << P_3_3 << " is not as expected";

    ASSERT_EQ(40, grid.getAggregate(P_1_1)) <<
        "Location aggregate in " << P_1_1 << " is not as expected";
    ASSERT_EQ(40, grid.getAggregate(P_3_3)) <<
        "Location aggregate in " << P_3_3 << " is not as expected";
}

TEST(Test_Graph2d, BasicWeigths)
{
    Grid2d grid(10);

    grid.setPayload(P_1_1, 10);
    grid.setPayload(P_3_3, 30);
    grid.setWeights(P_3_3, 100, 200, 300, 400);

    ASSERT_EQ(Grid2d::PAYLOAD_MIN, grid.getWeight(P_1_1, Grid2d::ORIENTATION_NORTH)) <<
        "North weight in location " << P_1_1 << " is not as expected";

    ASSERT_EQ(100, grid.getWeight(P_3_3, Grid2d::ORIENTATION_NORTH)) <<
        "North weight in location " << P_3_3 << " is not as expected";
}

TEST(Test_Graph2d, BasicClear)
{
    Grid2d grid(10);

    grid.setPayload(P_3_3, 30);
    grid.setWeights(P_3_3, 100, 200, 300, 400);

    ASSERT_EQ(30, grid.getPayload(P_3_3)) <<
        "Location payload in " << P_3_3 << " is not as expected";

    ASSERT_EQ(100, grid.getWeight(P_3_3, Grid2d::ORIENTATION_NORTH)) <<
        "North weight in location " << P_3_3 << " is not as expected";
    ASSERT_EQ(200, grid.getWeight(P_3_3, Grid2d::ORIENTATION_EAST)) <<
        "East weight in location " << P_3_3 << " is not as expected";
    ASSERT_EQ(300, grid.getWeight(P_3_3, Grid2d::ORIENTATION_SOUTH)) <<
        "South weight in location " << P_3_3 << " is not as expected";
    ASSERT_EQ(400, grid.getWeight(P_3_3, Grid2d::ORIENTATION_WEST)) <<
        "West weight in location " << P_3_3 << " is not as expected";

    grid.clear(P_3_3);

    ASSERT_EQ(Grid2d::PAYLOAD_MIN, grid.getPayload(P_3_3)) <<
        "Location payload in " << P_3_3 << " is not as expected";

    ASSERT_EQ(Grid2d::PAYLOAD_MIN, grid.getWeight(P_3_3, Grid2d::ORIENTATION_NORTH)) <<
        "North weight in location " << P_3_3 << " is not as expected";
    ASSERT_EQ(Grid2d::PAYLOAD_MIN, grid.getWeight(P_3_3, Grid2d::ORIENTATION_EAST)) <<
        "East weight in location " << P_3_3 << " is not as expected";
    ASSERT_EQ(Grid2d::PAYLOAD_MIN, grid.getWeight(P_3_3, Grid2d::ORIENTATION_SOUTH)) <<
        "South weight in location " << P_3_3 << " is not as expected";
    ASSERT_EQ(Grid2d::PAYLOAD_MIN, grid.getWeight(P_3_3, Grid2d::ORIENTATION_WEST)) <<
        "West weight in location " << P_3_3 << " is not as expected";

    ASSERT_EQ(Grid2d::PAYLOAD_MIN, grid.getPayload(P_2_2)) <<
        "Location payload in " << P_2_2 << " is not as expected";

    ASSERT_EQ(Grid2d::PAYLOAD_MIN, grid.getWeight(P_2_2, Grid2d::ORIENTATION_NORTH)) <<
        "North weight in location " << P_2_2 << " is not as expected";
    ASSERT_EQ(Grid2d::PAYLOAD_MIN, grid.getWeight(P_2_2, Grid2d::ORIENTATION_EAST)) <<
        "East weight in location " << P_2_2 << " is not as expected";
    ASSERT_EQ(Grid2d::PAYLOAD_MIN, grid.getWeight(P_3_3, Grid2d::ORIENTATION_SOUTH)) <<
        "South weight in location " << P_2_2 << " is not as expected";
    ASSERT_EQ(Grid2d::PAYLOAD_MIN, grid.getWeight(P_3_3, Grid2d::ORIENTATION_WEST)) <<
        "West weight in location " << P_2_2 << " is not as expected";

    grid.clear(P_2_2);
}
