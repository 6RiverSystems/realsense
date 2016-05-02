/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef GRID2D_HPP_
#define GRID2D_HPP_

#include <array>
#include <vector>
#include <unordered_map>
#include <tuple>
using namespace std;

// The hash function is required by C++ to convert a tuple into a
// acceptable map index.
namespace std
{
template<>
    struct hash<tuple<int, int>>
    {
        inline size_t operator()(const tuple<int, int>& node) const
        {
            int x;
            int y;
            tie(x, y) = node;

            return x * 1812433253 + y;
        }
};

} // namespace std

namespace srs {

class Grid2d
{
public:
    typedef tuple<int, int> NodeType;
    static array<NodeType, 4> ALLOWED_DIRECTIONS;

    inline static void node2Coordinates(NodeType node, int& x, int& y)
    {
        tie(x, y) = node;
    }

    inline static int getHeuristic(NodeType fromNode, NodeType toNode)
    {
        int x1;
        int y1;
        node2Coordinates(fromNode, x1, y1);

        int x2;
        int y2;
        node2Coordinates(toNode, x2, y2);

        return abs(x1 - x2) + abs(y1 - y2);
    }

    Grid2d(int width, int height) :
        width_(width),
        height_(height)
    {}

    ~Grid2d()
    {}

    inline void addValue(NodeType node, int value)
    {
        cells_[node] = value;
    }

    int getHeight()
    {
        return height_;
    }

    vector<NodeType> getNeighbors(NodeType node) const
    {
        int x;
        int y;
        node2Coordinates(node, x, y);

        int dx;
        int dy;
        vector<NodeType> results;

        for (auto neighbor : ALLOWED_DIRECTIONS)
        {
            node2Coordinates(neighbor, dx, dy);
            NodeType next(x + dx, y + dy);

            if (isWithinBounds(next))
            {
                results.push_back(next);
            }
        }

        return results;
    }

    int getWidth()
    {
        return width_;
    }

    inline int getValue(NodeType node)
    {
        return cells_[node];
    }

    inline bool isWithinBounds(NodeType node) const
    {
      int x;
      int y;
      node2Coordinates(node, x, y);

      return (0 <= x && x < width_) && (0 <= y && y < height_);
    }

private:
    int width_;
    int height_;

    unordered_map<NodeType, int> cells_;
};

array<Grid2d::NodeType, 4> Grid2d::ALLOWED_DIRECTIONS {
    NodeType( 1,  0),
    NodeType( 0, -1),
    NodeType(-1,  0),
    NodeType( 0,  1)
};

} // namespace srs

#endif // GRID2D_HPP_
