/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#include <gtest/gtest.h>
#include <ros/ros.h>

#include <vector>
using namespace std;

#include <srslib_framework/datastructure/MappedPriorityQueue.hpp>
#include <srslib_test/utils/MemoryWatch.hpp>
using namespace srs;

TEST(Test_MappedPriorityQueue, MappedPriorityQueueCreation)
{
    test::MemoryWatch memoryWatch;

    MappedPriorityQueue<int>* queue = new MappedPriorityQueue<int>();
    int item;

    ROS_DEBUG_STREAM("Memory usage: " << memoryWatch.getMemoryUsage());

    queue->push(500, 50);
    queue->push(100, 0);
    queue->push(100, 1);
    queue->push(400, 40);
    queue->push(200, 20);
    queue->push(500, 51);
    queue->push(500, 52);
    queue->push(300, 30);
    queue->push(400, 40);
    queue->push(500, 50);
    queue->push(500, 51);
    queue->push(500, 52);

    ROS_DEBUG_STREAM("POP ---------------------");
    ROS_DEBUG_STREAM(*queue);

    queue->pop(item);
    ROS_DEBUG_STREAM(item);

    queue->pop(item);
    ROS_DEBUG_STREAM(item);

    queue->pop(item);
    ROS_DEBUG_STREAM(item);

    queue->pop(item);
    ROS_DEBUG_STREAM(item);

    queue->pop(item);
    ROS_DEBUG_STREAM(item);

    ROS_DEBUG_STREAM("PUSH ---------------------");
    queue->push(500, 53);

    ROS_DEBUG_STREAM(*queue);

    ROS_DEBUG_STREAM("POP ---------------------");
    queue->pop(item);
    ROS_DEBUG_STREAM(item);

    ROS_DEBUG_STREAM("EXIST -------------------");
    ROS_DEBUG_STREAM(*queue);

    ROS_DEBUG_STREAM(queue->exists(50));
    ROS_DEBUG_STREAM(queue->exists(53));

    ROS_DEBUG_STREAM("ERASE -------------------");
    queue->erase(0);

    ROS_DEBUG_STREAM("CLEAR------------------");
    ROS_DEBUG_STREAM(*queue);

    while (!queue->empty())
    {
        queue->pop(item);
    }

    ROS_DEBUG_STREAM(*queue);

    delete queue;

    ROS_DEBUG_STREAM("Memory usage: " << memoryWatch.getMemoryUsage());
    ROS_DEBUG_STREAM("Memory leaks: " << !memoryWatch.isZero());
}
