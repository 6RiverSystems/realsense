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

struct Index
{
    int a;
    float b;

    friend ostream& operator<<(ostream& stream, const Index& index)
    {
        return stream << index.a << "-" << index.b;
    }

    bool operator==(const Index &other) const
    {
        if (this == &other)
        {
            return true;
        }

        return a == other.a;
    }
};

struct IndexHash
{
    size_t operator()(const Index& index) const
    {
        return hash<int>()(index.a);
    }
};

struct IndexEqual
{
    bool operator()(const Index& lhs, const Index& rhs) const
    {
        return lhs == rhs;
    }
};

TEST(Test_MappedPriorityQueue_IndexClass, Creation)
{
    test::MemoryWatch memoryWatch;

    MappedPriorityQueue<Index, float, IndexHash, IndexEqual>* queue =
        new MappedPriorityQueue<Index, float, IndexHash, IndexEqual>();

    queue->push(100.1, Index{6, 10});
    queue->push(500, Index{5, 50});
    queue->push(100, Index{1, 11});
    queue->push(200, Index{2, 20});
    queue->push(200, Index{3, 21});

    Index item;

    queue->pop(item);
    Index correct = Index{1, 11};
    ASSERT_EQ(correct, item) << "Unexpected item in the queue";

    queue->pop(item);
    correct = Index{6, 10};
    ASSERT_EQ(correct, item) << "Unexpected item in the queue";

    queue->pop(item);
    correct = Index{3, 21};
    ASSERT_EQ(correct, item) << "Unexpected item in the queue";

    queue->pop(item);
    correct = Index{2, 20};
    ASSERT_EQ(correct, item) << "Unexpected item in the queue";

    queue->pop(item);
    correct = Index{5, 50};
    ASSERT_EQ(correct, item) << "Unexpected item in the queue";

    queue->push(500, Index{100, 1000});

    ASSERT_EQ(1, queue->sizeOccuppiedBuckets()) << "Unexpected number of occupied buckets";
    ASSERT_EQ(queue->sizeInitialBuckets() - 1, queue->sizeEmptyBuckets()) <<
        "Unexpected number of empty buckets";

    delete queue;

    ASSERT_TRUE(memoryWatch.isZero()) << "Memory leaks occurred";
}

TEST(Test_MappedPriorityQueue_IndexClass, Exist)
{
    MappedPriorityQueue<Index, int, IndexHash, IndexEqual> queue;

    queue.push(100, Index{6, 10});
    queue.push(500, Index{5, 50});
    queue.push(100, Index{1, 11});
    queue.push(200, Index{2, 20});
    queue.push(200, Index{3, 21});

    ASSERT_TRUE(queue.exists(Index{6, 10})) << "Item not found in the queue";
    ASSERT_TRUE(queue.exists(Index{5, 50})) << "Item not found in the queue";
    ASSERT_FALSE(queue.exists(Index{99, 10})) << "Item found in the queue";
}

TEST(Test_MappedPriorityQueue_IndexClass, Erase)
{
    MappedPriorityQueue<Index, float, IndexHash, IndexEqual> queue;

    queue.push(100, Index{6, 10});
    queue.push(500, Index{5, 50});
    queue.push(100, Index{1, 11});
    queue.push(200, Index{2, 20});
    queue.push(200, Index{3, 21});

    ASSERT_TRUE(queue.exists(Index{1, 11})) << "Item not found in the queue";
    queue.erase(Index{1, 11});
    ASSERT_FALSE(queue.exists(Index{1, 11})) << "Item found in the queue";

    ASSERT_TRUE(queue.exists(Index{3, 21})) << "Item not found in the queue";
    queue.erase(Index{3, 21});
    ASSERT_FALSE(queue.exists(Index{3, 21})) << "Item found in the queue";

    Index item;

    queue.pop(item);
    Index correct = Index{6, 10};
    ASSERT_EQ(correct, item) << "Unexpected item in the queue";

    queue.pop(item);
    correct = Index{2, 20};
    ASSERT_EQ(correct, item) << "Unexpected item in the queue";

    queue.pop(item);
    correct = Index{5, 50};
    ASSERT_EQ(correct, item) << "Unexpected item in the queue";
}
