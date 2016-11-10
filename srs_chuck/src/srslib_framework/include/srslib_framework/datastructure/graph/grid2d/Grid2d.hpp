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
    using BaseType = char;

    static const BaseType PAYLOAD_NO_INFORMATION;
    static const BaseType PAYLOAD_MIN;
    static const BaseType PAYLOAD_MAX;

    static const BaseType WEIGHT_NO_INFORMATION;
    static const BaseType WEIGHT_MIN;
    static const BaseType WEIGHT_MAX;

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
            if (&lhs == &rhs)
            {
                return true;
            }

            return (lhs.x == rhs.x) && (lhs.y == rhs.y);
        }

        friend ostream& operator<<(ostream& stream, const Location& location)
        {
            return stream << location.toString();
        }

        string toString() const
        {
            stringstream stream;
            stream << "{" << x << ", " << y << "}";
            return stream.str();
        }

        int x;
        int y;
    };

    struct Position
    {
        Position(int x = 0, int y = 0, int orientation = 0) :
            x(x),
            y(y),
            orientation(orientation)
        {}

        Position(Location location, int orientation = 0) :
            x(location.x),
            y(location.y),
            orientation(orientation)
        {}

        ~Position()
        {}

        std::size_t hash() const
        {
            return x + 1812433253 * y + orientation;
        }

        friend bool operator==(const Position& lhs, const Position& rhs)
        {
            if (&lhs == &rhs)
            {
                return true;
            }

            return (lhs.x == rhs.x) && (lhs.y == rhs.y) && (lhs.orientation == rhs.orientation);
        }

        friend ostream& operator<<(ostream& stream, const Position& position)
        {
            return stream << position.toString();
        }

        string toString() const
        {
            stringstream stream;
            stream << "{" << x << ", " << y << ", " << orientation << "}";
            return stream.str();
        }

        int x;
        int y;
        int orientation;
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

    BaseType getAggregate(const Location& location) const;
    BaseType getAggregate(const Position& position) const;
    BaseType getPayload(const Location& location) const;
    BaseType getPayload(const Position& position) const;

    unsigned int getHeight() const
    {
        return height_;
    }

    bool getNeighbor(const Position& position, Position& result) const;

    unsigned int getOccupiedCount() const
    {
        return grid_.size();
    }

    BaseType getWeight(const Position& position) const;
    void getWeights(const Location& location,
        BaseType& north, BaseType& east, BaseType& south, BaseType& west) const;

    unsigned int getWeightCount() const
    {
        return weightCount_;
    }

    unsigned int getWidth() const
    {
        return width_;
    }

    bool isWithinBounds(const Location& location) const
    {
        return (0 <= location.x && location.x < width_) &&
            (0 <= location.y && location.y < height_);
    }

    bool isWithinBounds(const Position& position) const
    {
        return (0 <= position.x && position.x < width_) &&
            (0 <= position.y && position.y < height_);
    }

    void maxOnPayload(const Location& location, BaseType otherPayload);

    friend ostream& operator<<(ostream& stream, const Grid2d& grid);
    friend bool operator==(const Grid2d& lhs, const Grid2d& rhs);

    friend bool operator!=(const Grid2d& lhs, const Grid2d& rhs)
    {
        return !(lhs == rhs);
    }

    void setAggregateSize(unsigned int width, unsigned int height);
    void setPayload(const Location& location, BaseType newPayload);
    void setWeights(const Location& location,
        BaseType north, BaseType east, BaseType south, BaseType west);

private:
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
        Weights(BaseType north = WEIGHT_MIN, BaseType east = WEIGHT_MIN,
            BaseType south = WEIGHT_MIN, BaseType west = WEIGHT_MIN) :
                north(north),
                east(east),
                south(south),
                west(west)
        {}

        friend bool operator==(const Weights& lhs, const Weights& rhs)
        {
            return lhs.north == rhs.north &&
                lhs.east == rhs.east &&
                lhs.south == rhs.south &&
                lhs.west == rhs.west;
        }

        friend bool operator!=(const Weights& lhs, const Weights& rhs)
        {
            return !(lhs == rhs);
        }

        BaseType north;
        BaseType east;
        BaseType south;
        BaseType west;
    };

    struct Node
    {
        Node(Location location, BaseType payload, BaseType aggregate) :
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

        friend bool operator==(const Node& lhs, const Node& rhs)
        {
            if (&lhs == &rhs)
            {
                return true;
            }
            if (lhs.weights && !rhs.weights)
            {
                return false;
            }
            if (!lhs.weights && rhs.weights)
            {
                return false;
            }
            if (lhs.weights && *lhs.weights != *rhs.weights)
            {
                return false;
            }

            return lhs.aggregate == rhs.aggregate &&
                lhs.location == rhs.location &&
                lhs.payload == rhs.payload;
        }

        friend bool operator!=(const Node& lhs, const Node& rhs)
        {
            return !(lhs == rhs);
        }

        BaseType aggregate;

        const Location location;

        BaseType payload;

        Weights* weights;
    };

    using MapType = unordered_map<Location, Node*, Hash, EqualTo>;

    Node* addNode(const Location& location, BaseType payload, BaseType aggregate)
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

    void printGrid(ostream& stream, string title,
        std::function<BaseType (Node*)> fieldSelection) const;
    void print(ostream& stream, string title,
        std::function<BaseType (Node*)> fieldSelection) const;

    void updateAllAggregate();
    void updateNodeAggregate(Node* node, BaseType oldPayload, BaseType newPayload);
    void updatePayload(const Location& location, BaseType newPayload,
        std::function<BaseType (BaseType, BaseType)> payloadSelection);

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
            if (this == &other)
            {
                return true;
            }

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
