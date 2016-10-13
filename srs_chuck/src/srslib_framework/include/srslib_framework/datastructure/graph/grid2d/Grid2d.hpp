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

    enum {
        ORIENTATION_NORTH = 90,
        ORIENTATION_EAST = 0,
        ORIENTATION_SOUTH = 270,
        ORIENTATION_WEST = 180
    };

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
            return stream << "{" << location.x << ", " << location.y << "}";
        }

        int x;
        int y;
    };

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
            return lhs == rhs;
        }
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

    void maxCost(const Location& location, int cost);

    friend ostream& operator<<(ostream& stream, const Grid2d& grid);

    void setAggregateSize(unsigned int width, unsigned int height);
    void setCost(const Location& location, int newCost);
    void setWeights(const Location& location, int north, int east, int south, int west);

private:
    static constexpr int WIDTH = 4;

    struct Weights
    {
        Weights(
            int north = COST_MIN, int east = COST_MIN,
            int south = COST_MIN, int west = COST_MIN)
        {
            north = north;
            east = east;
            south = south;
            west = west;
        }

        int north;
        int east;
        int south;
        int west;
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
            stream << "Node "<< hex << reinterpret_cast<long>(node) << dec << " {" << endl;
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

    void printGrid(ostream& stream, string title, std::function<int (Node*)> fieldSelection) const;
    void printCosts(ostream& stream, string title, std::function<int (Node*)> fieldSelection) const;

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
