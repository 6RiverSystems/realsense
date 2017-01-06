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

#include <srslib_framework/datastructure/Location.hpp>
#include <srslib_framework/datastructure/Position.hpp>
#include <srslib_framework/math/BasicMath.hpp>

namespace srs {

class WeightedGrid2d
{
public:
    using BaseType = unsigned char;

    static const BaseType PAYLOAD_MIN;
    static const BaseType PAYLOAD_MAX;

    static const BaseType WEIGHT_MIN;
    static const BaseType WEIGHT_MAX;

    enum {
        ORIENTATION_NORTH = 90,
        ORIENTATION_EAST = 0,
        ORIENTATION_SOUTH = 270,
        ORIENTATION_WEST = 180
    };

    WeightedGrid2d(unsigned int size) :
        width_(size),
        height_(size)
    {}

    WeightedGrid2d(unsigned int width, unsigned int height) :
        width_(width),
        height_(height)
    {}

    ~WeightedGrid2d()
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
        return (0 <= position.location.x && position.location.x < width_) &&
            (0 <= position.location.y && position.location.y < height_);
    }

    void maxOnPayload(const Location& location, BaseType otherPayload);

    friend ostream& operator<<(ostream& stream, const WeightedGrid2d& grid);
    friend bool operator==(const WeightedGrid2d& lhs, const WeightedGrid2d& rhs);

    friend bool operator!=(const WeightedGrid2d& lhs, const WeightedGrid2d& rhs)
    {
        return !(lhs == rhs);
    }

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

    struct Node
    {
        Node(Location location,
            BaseType payload = PAYLOAD_MIN,
            BaseType north = WEIGHT_MIN, BaseType east = WEIGHT_MIN,
            BaseType south = WEIGHT_MIN, BaseType west = WEIGHT_MIN) :
            location(location),
            payload(payload),
            north(north),
            east(east),
            south(south),
            west(west)
        {}

        ~Node()
        {}

        friend ostream& operator<<(ostream& stream, const Node* node)
        {
            stream << "Node "<< hex << reinterpret_cast<long>(node) << dec << " {" << endl;
            stream << "l: " << node->location <<
                ", p: " << node->payload <<
                ", n: " << node->north <<
                ", e: " << node->east <<
                ", s: " << node->south <<
                ", w: " << node->west << "}";
            return stream;
        }

        inline friend bool operator==(const Node& lhs, const Node& rhs)
        {
            return lhs.location == rhs.location &&
                lhs.payload == rhs.payload &&
                lhs.north == rhs.north &&
                lhs.east == rhs.east &&
                lhs.south == rhs.south &&
                lhs.west == rhs.west;
        }

        inline friend bool operator!=(const Node& lhs, const Node& rhs)
        {
            return !(lhs == rhs);
        }

        inline bool zeroWeights()
        {
            return north == WEIGHT_MIN && east == WEIGHT_MIN &&
                south == WEIGHT_MIN && west == WEIGHT_MIN;
        }

        const Location location;

        BaseType payload;
        BaseType north;
        BaseType east;
        BaseType south;
        BaseType west;
    };

    using MapType = unordered_map<Location, Node*, Hash, EqualTo>;

    Node* addNode(const Location& location,
        BaseType payload,
        BaseType north, BaseType east,
        BaseType south, BaseType west)
    {
        Node* node = new Node(location, payload, north, east, south, west);
        grid_[location] = node;

        return node;
    }

    inline Node* findNode(const int& x, const int& y) const
    {
        auto found = grid_.find(Location(x, y));
        if (found != grid_.end())
        {
            return found->second;
        }

        return nullptr;
    }

    inline Node* findNode(const Location& location) const
    {
        auto found = grid_.find(location);
        if (found != grid_.end())
        {
            return found->second;
        }

        return nullptr;
    }

    inline Node* findNode(const Position& position) const
    {
        auto found = grid_.find(position.location);
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

    void updatePayload(const Location& location, BaseType newPayload,
        std::function<BaseType (BaseType, BaseType)> payloadSelection);

    MapType grid_;

    unsigned int height_;

    unsigned int width_;

public:
    struct const_iterator
    {
        const_iterator(WeightedGrid2d* grid, MapType::const_iterator mapIterator) :
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
        WeightedGrid2d* grid_;
        MapType::const_iterator mapIterator_;
    };

    const_iterator begin() const
    {
        return const_iterator(const_cast<WeightedGrid2d*>(this), grid_.begin());
    }

    const_iterator end() const
    {
        return const_iterator(const_cast<WeightedGrid2d*>(this), grid_.end());
    }
};

} // namespace srs
