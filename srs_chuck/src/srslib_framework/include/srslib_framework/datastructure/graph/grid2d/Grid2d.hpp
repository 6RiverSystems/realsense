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
    static const int PAYLOAD_MIN;
    static const int PAYLOAD_MAX;
    static const int WEIGHT_MIN;
    static const int WEIGHT_MAX;

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

        std::size_t hash() const
        {
            return x + 1812433253 * y;
        }

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

    Grid2d(unsigned int size) :
        width_(size),
        height_(size),
        aggregate_(false),
        aggregateHeight_(0),
        aggregateWidth_(0),
        weightCount_(0)
    {}

    Grid2d(unsigned int width, unsigned int height) :
        width_(width),
        height_(height),
        aggregate_(false),
        aggregateHeight_(0),
        aggregateWidth_(0),
        weightCount_(0)
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

    int getAggregate(const Location& location) const;
    int getPayload(const Location& location) const;

    unsigned int getHeight() const
    {
        return height_;
    }

    bool getNeighbor(const Location& location, int orientation, Location& result) const;

    unsigned int getOccupiedCount() const
    {
        return grid_.size();
    }

    int getWeight(const Location& location, int orientation) const;

    unsigned int getWeightCount() const
    {
        return weightCount_;
    }

    unsigned int getWidth() const
    {
        return width_;
    }

    void maxOnPayload(const Location& location, int otherPayload);

    friend ostream& operator<<(ostream& stream, const Grid2d& grid);

    void setAggregateSize(unsigned int width, unsigned int height);
    void setPayload(const Location& location, int newPayload);
    void setWeights(const Location& location, int north, int east, int south, int west);

private:
    static constexpr int WIDTH = 4;

    struct EqualTo
    {
        bool operator()(const Location& lhs, const Location& rhs) const
        {
            return lhs == rhs;
        }
    };

    struct Hash
    {
        std::size_t operator()(const Location& location) const
        {
            return location.hash();
        }
    };

    struct Weights
    {
        Weights(int north = WEIGHT_MIN, int east = WEIGHT_MIN,
            int south = WEIGHT_MIN, int west = WEIGHT_MIN) :
                north(north),
                east(east),
                south(south),
                west(west)
        {}

        int north;
        int east;
        int south;
        int west;
    };

    struct Node
    {
        Node(Location location, int payload, int aggregate) :
            location(location),
            payload(payload),
            aggregate(aggregate),
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
                ", p: " << node->payload <<
                ", a: " << node->aggregate << "\n}";
            return stream;
        }

        int aggregate;

        const Location location;

        int payload;

        Weights* weights;
    };

    using MapType = unordered_map<Location, Node*, Hash, EqualTo>;

    Node* addNode(const Location& location, int payload, int aggregate)
    {
        Node* node = new Node(location, payload, aggregate);
        grid_[location] = node;

        return node;
    }

    int calculateAggregate(Node* node);
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
    void print(ostream& stream, string title, std::function<int (Node*)> fieldSelection) const;

    void updateAllAggregate();
    void updateNodeAggregate(Node* node, int oldCost);
    void updatePayload(const Location& location, int newPayload,
        std::function<int (int, int)> payloadSelection);

    bool aggregate_;
    unsigned int aggregateWidth_;
    unsigned int aggregateHeight_;

    MapType grid_;

    unsigned int height_;

    unsigned int weightCount_;
    unsigned int width_;

public:
    struct const_iterator
    {
        const_iterator(Grid2d* grid, MapType::const_iterator mapIterator) :
            grid_(grid),
            mapIterator_(mapIterator)
        {}

        Location operator*()
        {
            return mapIterator_->first;
        }

        const_iterator& operator++()
        {
           mapIterator_++;
           return *this;
        }

        const_iterator& operator=(const const_iterator& other)
        {
            grid_ = other.grid_;
            mapIterator_ = other.mapIterator_;

            return *this;
        }

        bool operator==(const const_iterator& other) const
        {
            return mapIterator_ == other.mapIterator_;
        }

        bool operator!=(const const_iterator& other) const
        {
            return mapIterator_ != other.mapIterator_;
        }

    private:
        Grid2d* grid_;
        MapType::const_iterator mapIterator_;
    };

    const_iterator begin() const
    {
        return const_iterator(const_cast<Grid2d*>(this), grid_.begin());
    }

    const_iterator end() const
    {
        return const_iterator(const_cast<Grid2d*>(this), grid_.end());
    }
};

} // namespace srs
