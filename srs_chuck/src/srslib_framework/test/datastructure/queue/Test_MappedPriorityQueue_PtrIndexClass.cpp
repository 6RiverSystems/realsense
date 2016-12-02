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
    size_t operator()(Index* const& index) const
    {
        return hash<int>()(index->a);
    }
};

struct IndexEqual
{
    bool operator()(Index* const& lhs, Index* const& rhs) const
    {
        return *lhs == *rhs;
    }
};

TEST(Test_MappedPriorityQueue_PtrIndexClass, Creation)
{
    test::MemoryWatch memoryWatch;

    MappedPriorityQueue<Index*, float, IndexHash, IndexEqual>* queue =
        new MappedPriorityQueue<Index*, float, IndexHash, IndexEqual>();

    queue->push(100, new Index{6, 10});
    queue->push(500, new Index{5, 50});
    queue->push(100.1, new Index{1, 11});
    queue->push(200, new Index{2, 20});
    queue->push(200, new Index{3, 21});

    Index* item;

    queue->pop(item);
    Index correct = Index{6, 10};
    ASSERT_EQ(correct, *item) << "Unexpected item in the queue";
    delete item;

    queue->pop(item);
    correct = Index{1, 11};
    ASSERT_EQ(correct, *item) << "Unexpected item in the queue";
    delete item;

    queue->pop(item);
    correct = Index{3, 21};
    ASSERT_EQ(correct, *item) << "Unexpected item in the queue";
    delete item;

    queue->pop(item);
    correct = Index{2, 20};
    ASSERT_EQ(correct, *item) << "Unexpected item in the queue";
    delete item;

    queue->pop(item);
    correct = Index{5, 50};
    ASSERT_EQ(correct, *item) << "Unexpected item in the queue";
    delete item;

    delete queue;

    ASSERT_TRUE(memoryWatch.isZero()) << "Memory leaks occurred";
}

TEST(Test_MappedPriorityQueue_PtrIndexClass, Exist)
{
    MappedPriorityQueue<Index*, int, IndexHash, IndexEqual> queue;

    queue.push(100, new Index{6, 10});
    queue.push(500, new Index{5, 50});
    queue.push(100, new Index{1, 11});
    queue.push(200, new Index{2, 20});
    queue.push(200, new Index{3, 21});

    ASSERT_TRUE(queue.exists(new Index{6, 10})) << "Item not found in the queue";
    ASSERT_TRUE(queue.exists(new Index{5, 50})) << "Item not found in the queue";
    ASSERT_FALSE(queue.exists(new Index{99, 10})) << "Item found in the queue";
}

TEST(Test_MappedPriorityQueue_PtrIndexClass, Erase)
{
    MappedPriorityQueue<Index*, float, IndexHash, IndexEqual> queue;

    queue.push(100, new Index{6, 10});
    queue.push(500, new Index{5, 50});
    queue.push(100, new Index{1, 11});
    queue.push(200, new Index{2, 20});
    queue.push(200, new Index{3, 21});

    ASSERT_TRUE(queue.exists(new Index{1, 11})) << "Item not found in the queue";
    queue.erase(new Index{1, 11});
    ASSERT_FALSE(queue.exists(new Index{1, 11})) << "Item found in the queue";

    ASSERT_TRUE(queue.exists(new Index{3, 21})) << "Item not found in the queue";
    queue.erase(new Index{3, 21});
    ASSERT_FALSE(queue.exists(new Index{3, 21})) << "Item found in the queue";

    Index* item;

    queue.pop(item);
    Index correct = Index{6, 10};
    ASSERT_EQ(correct, *item) << "Unexpected item in the queue";

    queue.pop(item);
    correct = Index{2, 20};
    ASSERT_EQ(correct, *item) << "Unexpected item in the queue";

    queue.pop(item);
    correct = Index{5, 50};
    ASSERT_EQ(correct, *item) << "Unexpected item in the queue";
}
