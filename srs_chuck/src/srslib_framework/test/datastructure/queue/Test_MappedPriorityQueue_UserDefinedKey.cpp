/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#include <gtest/gtest.h>
#include <ros/ros.h>

#include <vector>
#include <unordered_map>
using namespace std;

#include <srslib_framework/datastructure/queue/MappedPriorityQueue.hpp>
using namespace srs;

template<typename T>
struct Node
{
    T a;
    T b;

    friend ostream& operator<<(ostream& stream, Node<T>& node)
    {
        return stream << node.a << "-" << node.b;
    }

    bool operator==(const Node<T> &other) const
    {
        return a == other.a && b == other.b;
    }
};

namespace std {
template<typename T>
struct hash<Node<T>>
{
    unsigned long operator()(const Node<T>& node) const
    {
        return hash<int>()(node.a + 101 * node.b);
    }
};
}

TEST(Test_MappedPriorityQueue, UserDefinedKey)
{
    MappedPriorityQueue<Node<int>, int, hash<Node<int>>> queue;

    queue.push(100, Node<int>{1, 10});
    queue.push(500, Node<int>{5, 50});
    queue.push(100, Node<int>{1, 11});
    queue.push(200, Node<int>{2, 20});
    queue.push(200, Node<int>{2, 21});

	ROS_DEBUG_STREAM(queue);

    Node<int> item;

	queue.pop(item);
	ROS_DEBUG_STREAM(item);

	ROS_DEBUG_STREAM(queue.exists(Node<int>{1, 11}));
	ROS_DEBUG_STREAM(queue.exists(Node<int>{5, 50}));

	queue.pop(item);
	ROS_DEBUG_STREAM(item);

	queue.pop(item);
	ROS_DEBUG_STREAM(item);

	queue.pop(item);
	ROS_DEBUG_STREAM(item);
}
