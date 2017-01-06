/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#include <gtest/gtest.h>
#include <ros/ros.h>

#include <vector>
using namespace std;

#include <srslib_framework/datastructure/graph/grid2d/SimpleGrid2d.hpp>
#include <srslib_framework/datastructure/Location.hpp>
#include <srslib_framework/datastructure/Position.hpp>
using namespace srs;

#include <srslib_test/datastructure/graph/grid2d/Grid2dUtils.hpp>

const Location P_1_1 = Location(1, 1);
const Location P_2_2 = Location(2, 2);
const Location P_3_3 = Location(3, 3);

TEST(Test_SimpleGraph2d, BasicSet)
{
    SimpleGrid2d grid(10);

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

TEST(Test_SimpleGraph2d, GetPayload)
{
    SimpleGrid2d grid(10);

    grid.setPayload(P_1_1, 10);
    grid.setPayload(P_3_3, 30);

    ASSERT_EQ(10, grid.getPayload(P_1_1)) <<
        "Location payload in " << P_1_1 << " is not as expected";
    ASSERT_EQ(30, grid.getPayload(P_3_3)) <<
        "Location payload in " << P_3_3 << " is not as expected";
    ASSERT_EQ(srs::SimpleGrid2d::PAYLOAD_NO_INFORMATION, grid.getPayload(P_2_2)) <<
        "Location payload in " << P_2_2 << " is not as expected";
}

TEST(Test_SimpleGraph2d, BasicMax)
{
    SimpleGrid2d grid(10);

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

    grid.maxOnPayload(P_2_2, srs::SimpleGrid2d::PAYLOAD_MAX);

    ASSERT_EQ(srs::SimpleGrid2d::PAYLOAD_MAX, grid.getPayload(P_2_2)) <<
        "Location payload in " << P_2_2 << " is not as expected";
}

TEST(Test_SimpleGraph2d, SetWithMax)
{
    SimpleGrid2d grid(10);

    grid.setPayload(P_1_1, 10);
    grid.setPayload(P_3_3, 30);

    ASSERT_EQ(10, grid.getPayload(P_1_1)) <<
        "Location payload in " << P_1_1 << " is not as expected";
    ASSERT_EQ(30, grid.getPayload(P_3_3)) <<
        "Location payload in " << P_3_3 << " is not as expected";

    grid.setPayload(P_3_3, srs::SimpleGrid2d::PAYLOAD_MAX);

    ASSERT_EQ(10, grid.getPayload(P_1_1)) <<
        "Location payload in " << P_1_1 << " is not as expected";
    ASSERT_EQ(srs::SimpleGrid2d::PAYLOAD_MAX, grid.getPayload(P_3_3)) <<
        "Location payload in " << P_3_3 << " is not as expected";
}

TEST(Test_SimpleGraph2d, AddWithMax)
{
    SimpleGrid2d grid(10);

    grid.setPayload(P_1_1, 10);
    grid.setPayload(P_3_3, 30);

    ASSERT_EQ(10, grid.getPayload(P_1_1)) <<
        "Location payload in " << P_1_1 << " is not as expected";
    ASSERT_EQ(30, grid.getPayload(P_3_3)) <<
        "Location payload in " << P_3_3 << " is not as expected";

    grid.maxOnPayload(P_3_3, srs::SimpleGrid2d::PAYLOAD_MAX);

    ASSERT_EQ(10, grid.getPayload(P_1_1)) <<
        "Location payload in " << P_1_1 << " is not as expected";
    ASSERT_EQ(srs::SimpleGrid2d::PAYLOAD_MAX, grid.getPayload(P_3_3)) <<
        "Location payload in " << P_3_3 << " is not as expected";
}

TEST(Test_SimpleGraph2d, BasicClear)
{
    SimpleGrid2d grid(10);

    grid.setPayload(P_3_3, 30);

    ASSERT_EQ(30, grid.getPayload(P_3_3)) <<
        "Location payload in " << P_3_3 << " is not as expected";

    grid.clear(P_3_3);

    ASSERT_EQ(SimpleGrid2d::PAYLOAD_NO_INFORMATION, grid.getPayload(P_3_3)) <<
        "Location payload in " << P_3_3 << " is not as expected";

    ASSERT_EQ(SimpleGrid2d::PAYLOAD_NO_INFORMATION, grid.getPayload(P_2_2)) <<
        "Location payload in " << P_2_2 << " is not as expected";
}

TEST(Test_SimpleGraph2d, BasicBeginEnd)
{
    SimpleGrid2d grid(10);

    SimpleGrid2d::const_iterator beginIt = grid.begin();
    SimpleGrid2d::const_iterator endIt = grid.end();

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

TEST(Test_SimpleGraph2d, BasicForEach)
{
    SimpleGrid2d grid(10);

    SimpleGrid2d::const_iterator beginIt = grid.begin();
    SimpleGrid2d::const_iterator endIt = grid.end();

    ASSERT_EQ(beginIt, endIt) << "The begin and end iterators do not match";

    grid.setPayload(P_3_3, 30);
    grid.setPayload(P_3_3, 30);
    grid.setPayload(P_2_2, 10);

    int payloads = 0;
    for (Location it : grid)
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

TEST(Test_SimpleGraph2d, OccupiedCount)
{
    SimpleGrid2d grid(10);

    grid.setPayload(P_3_3, 30);

    ASSERT_EQ(1, grid.getOccupiedCount()) << "Unexpected occupied cells count";

    grid.setPayload(P_3_3, 30);
    grid.setPayload(P_3_3, 30);

    ASSERT_EQ(1, grid.getOccupiedCount()) << "Unexpected occupied cells count";

    grid.setPayload(P_3_3, 30);
    grid.setPayload(P_2_2, 10);

    ASSERT_EQ(2, grid.getOccupiedCount()) << "Unexpected occupied cells count";

    grid.setPayload(P_2_2, SimpleGrid2d::PAYLOAD_NO_INFORMATION);
    grid.setPayload(P_1_1, SimpleGrid2d::PAYLOAD_NO_INFORMATION);

    ASSERT_EQ(1, grid.getOccupiedCount()) << "Unexpected occupied cells count";

    grid.maxOnPayload(P_2_2, SimpleGrid2d::PAYLOAD_MIN);

    ASSERT_EQ(2, grid.getOccupiedCount()) << "Unexpected occupied cells count";
}

TEST(Test_SimpleGraph2d, EqualOperator)
{
    SimpleGrid2d correct(10);
    correct.setPayload(P_3_3, 3);
    correct.setPayload(P_1_1, 1);
    correct.setPayload(P_2_2, 2);

    SimpleGrid2d grid1(10);
    grid1.setPayload(P_3_3, 3);
    grid1.setPayload(P_1_1, 1);
    grid1.setPayload(P_2_2, 2);

    ASSERT_EQ(grid1, grid1) << "The grid does not agree with itself";
    ASSERT_EQ(correct, grid1) << "The grids do not match";

    SimpleGrid2d grid2(10);
    grid2.setPayload(P_3_3, 30);

    ASSERT_NE(correct, grid2) << "The grids do match";
}

TEST(Test_SimpleGraph2d, CopyConstructor)
{
    SimpleGrid2d correct(10);
    correct.setPayload(P_3_3, 3);
    correct.setPayload(P_1_1, 1);
    correct.setPayload(P_2_2, 2);

    SimpleGrid2d grid1 = correct;

    ASSERT_EQ(grid1, grid1) << "The grid does not agree with itself";
    ASSERT_EQ(correct, grid1) << "The grids do not match";

    SimpleGrid2d grid2(grid1);

    ASSERT_EQ(grid2, grid2) << "The grid does not agree with itself";
    ASSERT_EQ(grid1, grid2) << "The grids do not match";
    ASSERT_EQ(correct, grid2) << "The grids do not match";
}

