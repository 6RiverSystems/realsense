/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <sstream>
#include <iostream>
#include <iomanip>
#include <vector>
#include <unordered_map>
#include <limits>
#include <functional>
using namespace std;

#include <srslib_framework/math/BasicMath.hpp>

namespace srs {

class Grid2d
{
public:
    static const int COST_MIN;
    static const int COST_MAX;

    static const int ORIENTATION_NORTH;
    static const int ORIENTATION_EAST;
    static const int ORIENTATION_SOUTH;
    static const int ORIENTATION_WEST;

    struct Location
    {
        Location(int x = 0, int y = 0) :
            x(x),
            y(y)
        {}

        friend bool operator==(const Location& lhs, const Location& rhs)
        {
            return (lhs.x == rhs.x) && (lhs.y == rhs.y);
        }

        friend ostream& operator<<(ostream& stream, const Location& location)
        {
            return stream << "<" << location.x << ", " << location.y << ">";
        }

        int x;
        int y;
    };

    Grid2d(unsigned int size) :
        width_(size),
        height_(size),
        aggregate_(false),
        aggregateHeight_(0),
        aggregateWidth_(0),
        hasWeights_(false)
    {}

    Grid2d(unsigned int width, unsigned int height) :
        width_(width),
        height_(height),
        aggregate_(false),
        aggregateHeight_(0),
        aggregateWidth_(0),
        hasWeights_(false)
    {}

    ~Grid2d()
    {
        clear();
    }

    void addCost(const Location& location, int cost);

    void clear();
    void clear(const Location& location);

    bool exists(const Location& location) const
    {
        if (!isWithinBounds(location))
        {
            return false;
        }

        return grid_.count(location);
    }

    bool isWithinBounds(const Location& location) const
    {
        return (0 <= location.x && location.x < width_) &&
            (0 <= location.y && location.y < height_);
    }

    int getAggregateCost(const Location& location) const;
    int getCost(const Location& location) const;

    unsigned int getHeight() const
    {
        return height_;
    }

    bool getNeighbor(const Location& location, int orientation, Location& result) const;
    int getWeight(const Location& location, int orientation) const;

    unsigned int getWidth() const
    {
        return width_;
    }

    friend ostream& operator<<(ostream& stream, const Grid2d& grid);

    void setAggregateSize(unsigned int width, unsigned int height);
    void setCost(const Location& location, int newCost);
    void setWeights(const Location& location, int north, int east, int south, int west);

private:
    static constexpr int WIDTH = 4;
    static constexpr int MAX_WEIGHT_DIRECTIONS = 4;

    struct LocationHash
    {
        std::size_t operator()(const Location& location) const
        {
            return location.x + 1812433253 * location.y;
        }
    };

    struct LocationEqual
    {
        bool operator()(const Location& lhs, const Location& rhs) const
        {
            return (lhs.x == rhs.x) && (lhs.y == rhs.y);
        }
    };

    struct Weights
    {
        Weights(
            int north = COST_MIN, int east = COST_MIN,
            int south = COST_MIN, int west = COST_MIN)
        {
            cost[ORIENTATION_NORTH] = north;
            cost[ORIENTATION_EAST] = east;
            cost[ORIENTATION_SOUTH] = south;
            cost[ORIENTATION_WEST] = west;
        }

        int cost[MAX_WEIGHT_DIRECTIONS];
    };

    struct Node
    {
        Node(Location location, int cost, int aggregateCost) :
            location(location),
            cost(cost),
            aggregateCost(aggregateCost),
            weights(nullptr)
        {}

        ~Node()
        {
            delete weights;
        }

        friend ostream& operator<<(ostream& stream, const Node* node)
        {
            stream << "Node "<< hex << reinterpret_cast<long>(node) << dec << " {" << '\n';
            stream << "l: " << node->location <<
                ", c: " << node->cost <<
                ", ac: " << node->aggregateCost << "\n}";
            return stream;
        }

        int aggregateCost;

        int cost;

        const Location location;

        Weights* weights;
    };

    Node* addNode(const Location& location, int cost, int aggregateCost)
    {
        Node* node = new Node(location, cost, aggregateCost);
        grid_[location] = node;

        return node;
    }

    int calculateAggregateCost(Node* node);
    void calculateAggregateArea(int x0, int y0, int& xi, int& xf, int& yi, int& yf);

    Node* findNode(const Location& location) const
    {
        auto found = grid_.find(location);
        if (found != grid_.end())
        {
            return found->second;
        }

        return nullptr;
    }

    void printGrid(string title, bool simple, ostream& stream,
        std::function<int (Node*)> fieldSelection) const;

    void updateAllAggregate();
    void updateNodeAggregate(Node* node, int oldCost);

    bool aggregate_;
    unsigned int aggregateWidth_;
    unsigned int aggregateHeight_;

    unordered_map<Location, Node*, LocationHash, LocationEqual> grid_;

    bool hasWeights_;
    unsigned int height_;

    unsigned int width_;
};

} // namespace srs
