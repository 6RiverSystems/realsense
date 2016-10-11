/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#include <gtest/gtest.h>
#include <ros/ros.h>

#include <vector>
#include <unordered_map>
#include <functional>
#include <unordered_set>
using namespace std;

#include <srslib_framework/datastructure/queue/MappedPriorityQueue.hpp>
#include <srslib_test/utils/MemoryWatch.hpp>
using namespace srs;

template<typename T>
struct Node
{
    T a;
    T b;

    friend ostream& operator<<(ostream& stream, const Node<T>& node)
    {
        return stream << node.a << "-" << node.b;
    }

    bool operator==(const Node<T> &other) const
    {
        return a == other.a && b == other.b;
    }

    size_t operator()(const Node<T>& node) const
    {
        return hash<T>()(node.a + 101 * node.b);
    }

    bool operator()(const Node<T>& lhs, const Node<T>& rhs) const
    {
        return lhs == rhs;
    }
};

TEST(Test_MappedPriorityQueue_InnerHashEqual, Creation)
{
    test::MemoryWatch memoryWatch;

    MappedPriorityQueue<Node<int>, int, Node<int>, Node<int>>* queue =
        new MappedPriorityQueue<Node<int>, int, Node<int>, Node<int>>();

    queue->push(100, Node<int>{1, 10});
    queue->push(500, Node<int>{5, 50});
    queue->push(100, Node<int>{1, 11});
    queue->push(200, Node<int>{2, 20});
    queue->push(200, Node<int>{2, 21});

    Node<int> item;

    queue->pop(item);
    Node<int> correct = Node<int>{1, 11};
    ASSERT_EQ(correct, item) << "Unexpected item in the queue";

    queue->pop(item);
    correct = Node<int>{1, 10};
    ASSERT_EQ(correct, item) << "Unexpected item in the queue";

    queue->pop(item);
    correct = Node<int>{2, 21};
    ASSERT_EQ(correct, item) << "Unexpected item in the queue";

    queue->pop(item);
    correct = Node<int>{2, 20};
    ASSERT_EQ(correct, item) << "Unexpected item in the queue";

    queue->pop(item);
    correct = Node<int>{5, 50};
    ASSERT_EQ(correct, item) << "Unexpected item in the queue";

    queue->push(500, Node<int>{100, 1000});

    ASSERT_EQ(1, queue->sizeOccuppiedBuckets()) << "Unexpected number of occupied buckets";
    ASSERT_EQ(queue->sizeInitialBuckets() - 1, queue->sizeEmptyBuckets()) <<
        "Unexpected number of empty buckets";

    delete queue;

    ASSERT_TRUE(memoryWatch.isZero()) << "Memory leaks occurred";
}

TEST(Test_MappedPriorityQueue_InnerHashEqual, Exist)
{
    test::MemoryWatch memoryWatch;

    MappedPriorityQueue<Node<int>, int, Node<int>, Node<int>>* queue =
        new MappedPriorityQueue<Node<int>, int, Node<int>, Node<int>>();

    queue->push(100, Node<int>{1, 10});
    queue->push(500, Node<int>{5, 50});
    queue->push(100, Node<int>{1, 11});
    queue->push(200, Node<int>{2, 20});
    queue->push(200, Node<int>{2, 21});

    ASSERT_TRUE(queue->exists(Node<int>{1, 10})) << "Item not found in the queue";
    ASSERT_TRUE(queue->exists(Node<int>{5, 50})) << "Item not found in the queue";
    ASSERT_FALSE(queue->exists(Node<int>{99, 10})) << "Item found in the queue";

    ASSERT_EQ(3, queue->sizeOccuppiedBuckets()) << "Unexpected number of occupied buckets";
    ASSERT_EQ(queue->sizeInitialBuckets() - 3, queue->sizeEmptyBuckets()) <<
        "Unexpected number of empty buckets";

    delete queue;

    ASSERT_TRUE(memoryWatch.isZero()) << "Memory leaks occurred";
}

TEST(Test_MappedPriorityQueue_InnerHashEqual, Erase)
{
    test::MemoryWatch memoryWatch;

    MappedPriorityQueue<Node<int>, int, Node<int>, Node<int>>* queue =
        new MappedPriorityQueue<Node<int>, int, Node<int>, Node<int>>();

    queue->push(100, Node<int>{1, 10});
    queue->push(500, Node<int>{5, 50});
    queue->push(100, Node<int>{1, 11});
    queue->push(200, Node<int>{2, 20});
    queue->push(200, Node<int>{2, 21});

    ASSERT_TRUE(queue->exists(Node<int>{5, 50})) << "Item not found in the queue";
    queue->erase(Node<int>{5, 50});
    ASSERT_FALSE(queue->exists(Node<int>{5, 50})) << "Item found in the queue";

    Node<int> item;
    Node<int> correct = Node<int>{1, 11};
    queue->pop(item);
    ASSERT_EQ(correct, item) << "Unexpected item in the queue";

    ASSERT_EQ(2, queue->sizeOccuppiedBuckets()) << "Unexpected number of occupied buckets";
    ASSERT_EQ(queue->sizeInitialBuckets() - 2, queue->sizeEmptyBuckets()) <<
        "Unexpected number of empty buckets";

    delete queue;

    ASSERT_TRUE(memoryWatch.isZero()) << "Memory leaks occurred";
}
