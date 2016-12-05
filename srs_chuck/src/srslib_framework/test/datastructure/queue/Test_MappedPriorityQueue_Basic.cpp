/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#include <gtest/gtest.h>
#include <ros/ros.h>

#include <vector>
using namespace std;

#include <srslib_framework/datastructure/queue/MappedPriorityQueue.hpp>
#include <srslib_test/utils/MemoryWatch.hpp>
using namespace srs;

TEST(Test_MappedPriorityQueue, Creation)
{
    test::MemoryWatch memoryWatch;

    MappedPriorityQueue<int>* queue = new MappedPriorityQueue<int>();

    queue->push(500, 50);
    queue->push(100, 0);
    queue->push(400, 40);
    queue->push(500, 51);
    queue->push(500, 52);
    queue->push(300, 30);
    queue->push(100, 1);
    queue->push(500, 50);
    queue->push(500, 51);
    queue->push(500, 52);
    queue->push(200, 20);

    int item;
    queue->pop(item);
    ASSERT_EQ(0, item) << "Unexpected item in the queue";

    queue->pop(item);
    ASSERT_EQ(1, item) << "Unexpected item in the queue";

    queue->pop(item);
    ASSERT_EQ(20, item) << "Unexpected item in the queue";

    queue->pop(item);
    ASSERT_EQ(30, item) << "Unexpected item in the queue";

    queue->pop(item);
    ASSERT_EQ(40, item) << "Unexpected item in the queue";

    queue->push(500, 53);

    queue->pop(item);
    ASSERT_EQ(50, item) << "Unexpected item in the queue";

    ASSERT_EQ(1, queue->sizeOccuppiedBuckets()) << "Unexpected number of occupied buckets";
    ASSERT_EQ(queue->sizeInitialBuckets() - 1, queue->sizeEmptyBuckets()) <<
        "Unexpected number of empty buckets";

    delete queue;

    ASSERT_TRUE(memoryWatch.isZero()) << "Memory leaks occurred";
}

TEST(Test_MappedPriorityQueue, Exist)
{
    test::MemoryWatch memoryWatch;

    MappedPriorityQueue<int>* queue = new MappedPriorityQueue<int>();

    queue->push(500, 50);
    queue->push(100, 0);
    queue->push(100, 1);
    queue->push(400, 40);
    queue->push(200, 20);
    queue->push(500, 51);
    queue->push(500, 52);
    queue->push(300, 30);
    queue->push(500, 50);
    queue->push(500, 51);
    queue->push(500, 52);

    ASSERT_TRUE(queue->exists(50)) << "Item not found in the queue";
    ASSERT_TRUE(queue->exists(1)) << "Item not found in the queue";
    ASSERT_FALSE(queue->exists(999)) << "Item found in the queue";

    ASSERT_EQ(5, queue->sizeOccuppiedBuckets()) << "Unexpected number of occupied buckets";
    ASSERT_EQ(queue->sizeInitialBuckets() - 5, queue->sizeEmptyBuckets()) <<
        "Unexpected number of empty buckets";

    delete queue;

    ASSERT_TRUE(memoryWatch.isZero()) << "Memory leaks occurred";
}

TEST(Test_MappedPriorityQueue, Erase)
{
    test::MemoryWatch memoryWatch;

    MappedPriorityQueue<int>* queue = new MappedPriorityQueue<int>();

    queue->push(500, 50);
    queue->push(100, 0);
    queue->push(100, 1);
    queue->push(400, 40);
    queue->push(400, 41);
    queue->push(200, 20);
    queue->push(500, 51);
    queue->push(500, 52);
    queue->push(300, 30);
    queue->push(500, 50);
    queue->push(500, 51);
    queue->push(500, 52);

    ASSERT_TRUE(queue->exists(0)) << "Item not found in the queue";
    queue->erase(0);
    ASSERT_FALSE(queue->exists(0)) << "Item found in the queue";

    int item;
    queue->pop(item);
    ASSERT_EQ(1, item) << "Unexpected item in the queue";

    ASSERT_TRUE(queue->exists(40)) << "Item not found in the queue";
    queue->erase(40);
    ASSERT_FALSE(queue->exists(40)) << "Item found in the queue";

    queue->pop(item);
    ASSERT_EQ(20, item) << "Unexpected item in the queue";

    ASSERT_EQ(3, queue->sizeOccuppiedBuckets()) << "Unexpected number of occupied buckets";
    ASSERT_EQ(queue->sizeInitialBuckets() - 3, queue->sizeEmptyBuckets()) <<
        "Unexpected number of empty buckets";

    delete queue;

    ASSERT_TRUE(memoryWatch.isZero()) << "Memory leaks occurred";
}

TEST(Test_MappedPriorityQueue, Clear)
{
    test::MemoryWatch memoryWatch;

    MappedPriorityQueue<int>* queue = new MappedPriorityQueue<int>();

    queue->push(500, 50);
    queue->push(100, 0);
    queue->push(100, 1);
    queue->push(400, 40);
    queue->push(200, 20);
    queue->push(500, 51);
    queue->push(500, 52);
    queue->push(300, 30);
    queue->push(500, 50);
    queue->push(500, 51);
    queue->push(500, 52);

    ASSERT_EQ(5, queue->sizeOccuppiedBuckets()) << "Unexpected number of occupied buckets";
    ASSERT_EQ(queue->sizeInitialBuckets() - 5, queue->sizeEmptyBuckets()) <<
        "Unexpected number of empty buckets";
    ASSERT_FALSE(queue->empty()) << "The queue is empty";

    queue->clear();

    ASSERT_EQ(0, queue->sizeOccuppiedBuckets()) << "Unexpected number of occupied buckets";
    ASSERT_EQ(queue->sizeInitialBuckets(), queue->sizeEmptyBuckets()) <<
        "Unexpected number of empty buckets";
    ASSERT_TRUE(queue->empty()) << "The queue is not empty";

    delete queue;

    ASSERT_TRUE(memoryWatch.isZero()) << "Memory leaks occurred";
}
