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
using namespace std;

#include <srslib_framework/math/BasicMath.hpp>

namespace srs {

class Grid2d
{
public:
    static const int MIN_COST;
    static const int MAX_COST;

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
        aggregateWidth_(0)
    {}

    Grid2d(unsigned int width, unsigned int height) :
        width_(width),
        height_(height),
        aggregate_(false),
        aggregateHeight_(0),
        aggregateWidth_(0)
    {}

    ~Grid2d()
    {
        clear();
    }

    void addCost(const Location& location, int cost);

    void clear();

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

    unsigned int getWidth() const
    {
        return width_;
    }

    friend ostream& operator<<(ostream& stream, const Grid2d& grid);

    void setAggregateSize(unsigned int width, unsigned int height);
    void setCost(const Location& location, int newCost);

private:
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

    struct Node
    {
        Node(Location location, int cost, int aggregateCost) :
            location(location),
            cost(cost),
            aggregateCost(aggregateCost)
        {}

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
    };

    int calculateAggregateCost(Node* node);
    void calculateAggregateArea(int x0, int y0, int& xi, int& xf, int& yi, int& yf);

    void updateAllAggregate();
    void updateNodeAggregate(Node* node, int oldCost);

    bool aggregate_;
    unsigned int aggregateWidth_;
    unsigned int aggregateHeight_;

    unordered_map<Location, Node*, LocationHash, LocationEqual> grid_;

    unsigned int height_;

    unsigned int width_;
};

} // namespace srs
