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
#include <srslib_framework/datastructure/graph/grid2d/BaseGrid2d.hpp>
#include <srslib_framework/math/BasicMath.hpp>

namespace srs {

class SimpleGrid2d: public BaseGrid2d
{
public:
    static const BaseType PAYLOAD_NO_INFORMATION;
    static const BaseType PAYLOAD_MIN;
    static const BaseType PAYLOAD_MAX;

    SimpleGrid2d(unsigned int size) :
        BaseGrid2d(size)
    {}

    SimpleGrid2d(unsigned int width, unsigned int height) :
        BaseGrid2d(width, height)
    {}

    SimpleGrid2d(const SimpleGrid2d& other);

    ~SimpleGrid2d()
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

    BaseType getMinPayloadValue()
    {
        return PAYLOAD_MIN;
    }

    BaseType getMaxPayloadValue()
    {
        return PAYLOAD_MAX;
    }

    BaseType getPayload(const Location& location) const;
    BaseType getPayload(const Position& position) const;

    unsigned int getOccupiedCount() const
    {
        return grid_.size();
    }

    void payloadMax(const Location& location, BaseType otherPayload);
    void payloadSet(const Location& location, BaseType newPayload);

    friend ostream& operator<<(ostream& stream, const SimpleGrid2d& grid);
    friend bool operator==(const SimpleGrid2d& lhs, const SimpleGrid2d& rhs);

    friend bool operator!=(const SimpleGrid2d& lhs, const SimpleGrid2d& rhs)
    {
        return !(lhs == rhs);
    }

protected:
    struct Node
    {
        Node(Location location, BaseType payload) :
            location(location),
            payload(payload)
        {}

        ~Node()
        {}

        friend ostream& operator<<(ostream& stream, const Node* node)
        {
            stream << "Node "<< hex << reinterpret_cast<long>(node) << dec << " {" << endl;
            stream << "l: " << node->location <<
                ", p: " << node->payload << "\n}";
            return stream;
        }

        friend bool operator==(const Node& lhs, const Node& rhs)
        {
            return lhs.location == rhs.location &&
                lhs.payload == rhs.payload;
        }

        friend bool operator!=(const Node& lhs, const Node& rhs)
        {
            return !(lhs == rhs);
        }

        const Location location;

        BaseType payload;
    };

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

    using MapType = unordered_map<Location, Node*, Hash, EqualTo>;

    void addNode(const Location& location, BaseType payload)
    {
        Node* node = new Node(location, payload);
        grid_[location] = node;
    }

    void printGrid(ostream& stream, string title,
        std::function<BaseType (Node*)> fieldSelection) const;
    void print(ostream& stream, string title,
        std::function<BaseType (Node*)> fieldSelection) const;

    void updatePayload(const Location& location, BaseType newPayload,
        std::function<BaseType (BaseType, BaseType)> payloadSelection);

    MapType grid_;

public:
    struct const_iterator
    {
        const_iterator(SimpleGrid2d* grid, MapType::const_iterator mapIterator) :
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
        SimpleGrid2d* grid_;
        MapType::const_iterator mapIterator_;
    };

    const_iterator begin() const
    {
        return const_iterator(const_cast<SimpleGrid2d*>(this), grid_.begin());
    }

    const_iterator end() const
    {
        return const_iterator(const_cast<SimpleGrid2d*>(this), grid_.end());
    }
};

} // namespace srs
