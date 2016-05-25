/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#include <gtest/gtest.h>

#include <vector>
#include <unordered_map>
using namespace std;

#include <srslib_framework/graph/grid2d/Grid2d.hpp>
#include <srslib_framework/datastructure/MappedPriorityQueue.hpp>
#include <srslib_framework/search/SearchNode.hpp>
#include <srslib_framework/search/SearchAction.hpp>
#include <srslib_framework/search/SearchPosition.hpp>
using namespace srs;

TEST(Test_MappedPriorityQueue, UserDefinedKey)
{
    typedef typename Grid2d::LocationType LocationType;
    typedef SearchNode<Grid2d> SearchNodeType;
    typedef SearchAction<Grid2d> SearchActionType;

    MappedPriorityQueue<SearchNodeType*, unsigned int> queue;

    SearchPosition<Grid2d> position1 = SearchPosition<Grid2d>(Grid2d::LocationType(30, 19), 0);
    SearchActionType action1 = SearchActionType(SearchActionType::NONE, position1);

    SearchPosition<Grid2d> position2 = SearchPosition<Grid2d>(Grid2d::LocationType(28, 19), 180);
    SearchActionType action2 = SearchActionType(SearchActionType::NONE, position2);

    SearchPosition<Grid2d> position3 = SearchPosition<Grid2d>(Grid2d::LocationType(28, 19), 0);
    SearchActionType action3 = SearchActionType(SearchActionType::NONE, position3);

    SearchPosition<Grid2d> position4 = SearchPosition<Grid2d>(Grid2d::LocationType(30, 19), 180);
    SearchActionType action4 = SearchActionType(SearchActionType::NONE, position4);

    SearchNodeType* node1 = new SearchNodeType(action1, nullptr);
    SearchNodeType* node2 = new SearchNodeType(action2, nullptr);
    SearchNodeType* node3 = new SearchNodeType(action3, nullptr);
    SearchNodeType* node4 = new SearchNodeType(action4, nullptr);

    queue.push(154, node4);
    queue.push(111, node1);
    queue.push(150, node3);
    queue.push(115, node2);

    cout << queue << endl;

    SearchNodeType* item;
    queue.pop(item);

    cout << queue << endl;
}
