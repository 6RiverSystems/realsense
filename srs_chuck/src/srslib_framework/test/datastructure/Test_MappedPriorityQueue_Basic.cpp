/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#include <gtest/gtest.h>

#include <vector>
using namespace std;

#include <srslib_framework/datastructure/MappedPriorityQueue.hpp>
using namespace srs;

TEST(Test_MappedPriorityQueue, MappedPriorityQueueCreation)
{
    MappedPriorityQueue<int> queue;

    queue.push(400, 40);
    queue.push(500, 50);
    queue.push(100, 0);
    queue.push(100, 1);
    queue.push(400, 40);
    queue.push(200, 20);
    queue.push(500, 51);
    queue.push(500, 52);
    queue.push(300, 30);
    queue.push(400, 40);
    queue.push(500, 50);
    queue.push(500, 51);
    queue.push(500, 52);

    cout << "---------------------" << endl;
    cout << queue << endl;

    cout << "---------------------" << endl;
    int item;

    queue.pop(item);
    cout << item << endl;

    queue.pop(item);
    cout << item << endl;

    queue.pop(item);
    cout << item << endl;

    queue.pop(item);
    cout << item << endl;

    queue.pop(item);
    cout << item << endl;

    cout << "---------------------" << endl;
    queue.push(500, 53);

    cout << queue << endl;

    cout << "---------------------" << endl;
    queue.pop(item);
    cout << item << endl;

    cout << "---------------------" << endl;
    cout << queue << endl;

    cout << queue.exists(50) << endl;
    cout << queue.exists(53) << endl;

    queue.erase(0);

    cout << "---------------------" << endl;
    cout << queue << endl;
}
