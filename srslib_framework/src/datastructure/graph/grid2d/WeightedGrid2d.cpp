#include <srslib_framework/datastructure/graph/grid2d/WeightedGrid2d.hpp>

#include <srslib_framework/datastructure/graph/grid2d/OutOfRangeException.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

const WeightedGrid2d::BaseType WeightedGrid2d::PAYLOAD_MIN = 0;
const WeightedGrid2d::BaseType WeightedGrid2d::PAYLOAD_MAX = 255;

const WeightedGrid2d::BaseType WeightedGrid2d::WEIGHT_MIN = 0;
const WeightedGrid2d::BaseType WeightedGrid2d::WEIGHT_MAX = 255;

////////////////////////////////////////////////////////////////////////////////////////////////////
WeightedGrid2d::WeightedGrid2d(const WeightedGrid2d& other) :
    BaseGrid2d(other.getWidth(), other.getHeight())
{
    for (auto node : other.grid_)
    {
        Node* internalNode = node.second;
        addNode(internalNode->location,
            internalNode->payload,
            internalNode->north, internalNode->northEast,
            internalNode->east, internalNode->southEast,
            internalNode->south, internalNode->southWest,
            internalNode->west, internalNode->northWest);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void WeightedGrid2d::clear()
{
    for (auto node : grid_)
    {
        delete node.second;
    }

    grid_.clear();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void WeightedGrid2d::clear(const Location& location)
{
    auto found = grid_.find(location);
    if (found != grid_.end())
    {
        delete found->second;
        grid_.erase(found);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
WeightedGrid2d::BaseType WeightedGrid2d::getPayload(const Location& location) const
{
    Node* node = findNode(location);
    if (node)
    {
        return node->payload;
    }

    return PAYLOAD_MIN;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
WeightedGrid2d::BaseType WeightedGrid2d::getPayload(const Position& position) const
{
    Node* node = findNode(position);
    if (node)
    {
        return node->payload;
    }

    return PAYLOAD_MIN;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
WeightedGrid2d::BaseType WeightedGrid2d::getWeight(const Position& position) const
{
    Node* node = findNode(position);
    if (node)
    {
        switch (position.orientation)
        {
            case ORIENTATION_NORTH:
                return node->north;

            case ORIENTATION_NORTH_EAST:
                return node->northEast;

            case ORIENTATION_EAST:
                return node->east;

            case ORIENTATION_SOUTH_EAST:
                return node->southEast;

            case ORIENTATION_SOUTH:
                return node->south;

            case ORIENTATION_SOUTH_WEST:
                return node->southWest;

            case ORIENTATION_WEST:
                return node->west;

            case ORIENTATION_NORTH_WEST:
                return node->northWest;
        }
    }

    return WEIGHT_MIN;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void WeightedGrid2d::getWeights(const Location& location,
    BaseType& north, BaseType& northEast,
    BaseType& east, BaseType& southEast,
    BaseType& south, BaseType& southWest,
    BaseType& west, BaseType& northWest) const
{
    Node* node = findNode(location);
    if (node)
    {
        north = node->north;
        northEast = node->northEast;
        east = node->east;
        southEast = node->southEast;
        south = node->south;
        southWest = node->southWest;
        west = node->west;
        northWest = node->northWest;
    }
    else
    {
        north = WEIGHT_MIN;
        northEast = WEIGHT_MIN;
        east = WEIGHT_MIN;
        southEast = WEIGHT_MIN;
        south = WEIGHT_MIN;
        southWest = WEIGHT_MIN;
        west = WEIGHT_MIN;
        northWest = WEIGHT_MIN;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void WeightedGrid2d::payloadMax(const Location& location, BaseType otherPayload)
{
    updatePayload(location, otherPayload,
        [] (BaseType oldPayload, BaseType newPayload) -> BaseType
        {
            return max(oldPayload, newPayload);
        }
    );
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void WeightedGrid2d::payloadSet(const Location& location, BaseType newPayload)
{
    updatePayload(location, newPayload,
        [] (BaseType oldPayload, BaseType newPayload) -> BaseType
        {
            return newPayload;
        }
    );
}

////////////////////////////////////////////////////////////////////////////////////////////////////
ostream& operator<<(ostream& stream, const WeightedGrid2d& grid)
{
    stream << "Grid2d {" << endl;

    stream << "(" << grid.getHeight() << "x" << grid.getWidth() << ")" << endl;

    grid.printGrid(stream, "Simplified",
        [] (WeightedGrid2d::Node* node) -> WeightedGrid2d::BaseType
        {
            return node->payload;
        }
    );

    grid.print(stream, "Payload",
        [] (WeightedGrid2d::Node* node) -> WeightedGrid2d::BaseType
        {
            return node->payload;
        }
    );

    grid.print(stream, "North Weights",
        [] (WeightedGrid2d::Node* node) -> WeightedGrid2d::BaseType
        {
            return node->north;
        }
    );

    grid.print(stream, "North-East Weights",
        [] (WeightedGrid2d::Node* node) -> WeightedGrid2d::BaseType
        {
            return node->northEast;
        }
    );

    grid.print(stream, "East Weights",
        [] (WeightedGrid2d::Node* node) -> WeightedGrid2d::BaseType
        {
            return node->east;
        }
    );

    grid.print(stream, "South-East Weights",
        [] (WeightedGrid2d::Node* node) -> WeightedGrid2d::BaseType
        {
            return node->southEast;
        }
    );

    grid.print(stream, "South Weights",
        [] (WeightedGrid2d::Node* node) -> WeightedGrid2d::BaseType
        {
            return node->south;
        }
    );

    grid.print(stream, "South-West Weights",
        [] (WeightedGrid2d::Node* node) -> WeightedGrid2d::BaseType
        {
            return node->southWest;
        }
    );

    grid.print(stream, "West Weights",
        [] (WeightedGrid2d::Node* node) -> WeightedGrid2d::BaseType
        {
            return node->west;
        }
    );

    grid.print(stream, "North-West Weights",
        [] (WeightedGrid2d::Node* node) -> WeightedGrid2d::BaseType
        {
            return node->northWest;
        }
    );

    stream << "}";

    return stream;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
bool operator==(const WeightedGrid2d& lhs, const WeightedGrid2d& rhs)
{
    // First check addresses
    if (&lhs == &rhs)
    {
        return true;
    }

    // Check that the dimensions are equal
    if (lhs.getWidth() != rhs.getWidth())
    {
        return false;
    }
    if (lhs.getHeight() != rhs.getHeight())
    {
        return false;
    }

    // Check that the grids contain the same number of items
    if (lhs.getOccupiedCount() != rhs.getOccupiedCount())
    {
        return false;
    }

    // Check that the grids contain the same items
    for (auto it : lhs.grid_)
    {
        WeightedGrid2d::Node* node = rhs.findNode(it.first);
        if (!node || *it.second != *node)
        {
            return false;
        }
    }

    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void WeightedGrid2d::setWeights(const Location& location,
    BaseType north, BaseType northEast,
    BaseType east, BaseType southEast,
    BaseType south, BaseType southWest,
    BaseType west, BaseType northWest)
{
    Node* node = findNode(location);

    bool nonZeroWeights = north != WEIGHT_MIN || northEast != WEIGHT_MIN ||
        east != WEIGHT_MIN || southEast != WEIGHT_MIN ||
        south != WEIGHT_MIN || southWest != WEIGHT_MIN ||
        west != WEIGHT_MIN || northWest != WEIGHT_MIN;

    if (!node)
    {
        if (!nonZeroWeights)
        {
            return;
        }
        addNode(location, PAYLOAD_MIN,
            north, northEast,
            east, southEast,
            south, southWest,
            west, northWest);
    }
    else
    {
        node->north = north;
        node->northEast = northEast;
        node->east = east;
        node->southEast = southEast;
        node->south = south;
        node->southWest = southWest;
        node->west = west;
        node->northWest = northWest;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

////////////////////////////////////////////////////////////////////////////////////////////////////
void WeightedGrid2d::printGrid(ostream& stream, string title,
    std::function<BaseType (Node*)> fieldSelection) const
{
    if (!title.empty())
    {
        stream << endl << title << endl;
    }

    size_t maxHeight = getHeight() - 1;
    for (size_t y = 0; y < getHeight(); ++y)
    {
        for (size_t x = 0; x < getWidth(); ++x)
        {
            WeightedGrid2d::Node* node = findNode(x, maxHeight-y);
            if (node)
            {
                BaseType payload = fieldSelection(node);

                if (payload == WeightedGrid2d::PAYLOAD_MAX)
                {
                    stream << "# ";
                }
                else if (payload == WeightedGrid2d::PAYLOAD_MIN)
                {
                    if (node->zeroWeights())
                    {
                        stream << ". ";
                    }
                    else
                    {
                        stream << "' ";
                    }
                }
                else
                {
                    stream << "+ ";
                }
            }
            else
            {
                stream << "? ";
            }
        }
        stream << endl;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void WeightedGrid2d::print(ostream& stream, string title,
    std::function<BaseType (Node*)> fieldSelection) const
{
    if (!title.empty())
    {
        stream << endl << title << endl;
    }

    const int WIDTH = 4;
    stream << right << setw(WIDTH) << ' ';
    for (size_t x = 0; x < getWidth(); ++x)
    {
        stream << right << setw(WIDTH) << x;
    }
    stream << endl;

    size_t maxHeight = getHeight() - 1;
    for (size_t y = 0; y < getHeight(); ++y)
    {
        stream << right << setw(WIDTH) << y;
        for (size_t x = 0; x < getWidth(); ++x)
        {
            WeightedGrid2d::Node* node = findNode(x, maxHeight-y);
            if (node)
            {
                BaseType info = fieldSelection(node);

                if (info == WeightedGrid2d::PAYLOAD_MAX)
                {
                    stream << right << setw(WIDTH) << "#";
                }
                else if (info == WeightedGrid2d::PAYLOAD_MIN)
                {
                    stream << right << setw(WIDTH) << ".";
                }
                else
                {
                    stream << right << setw(WIDTH) << static_cast<int>(info);
                }
            }
            else
            {
                stream << right << setw(WIDTH) << "?";
            }
        }
        stream << endl;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void WeightedGrid2d::updatePayload(const Location& location, BaseType newPayload,
    std::function<BaseType (BaseType, BaseType)> payloadSelection)
{
    if (!isWithinBounds(location))
    {
        throw OutOfRangeException(location, Location(getWidth(), getHeight()));
    }

    BaseType finalPayload;
    Node* node = findNode(location);
    if (node)
    {
        finalPayload = payloadSelection(node->payload, newPayload);
        node->payload = finalPayload;

        // Remove the node if the specified payload
        // is the minimum available payload and there is no additional
        // information in the weights
        if (finalPayload == PAYLOAD_MIN && node->zeroWeights())
        {
            clear(location);
        }
    }
    else
    {
        finalPayload = payloadSelection(PAYLOAD_MIN, newPayload);

        // Add a new node only if the specified payload
        // is not the minimum available payload
        if (finalPayload != PAYLOAD_MIN)
        {
            addNode(location, finalPayload,
                WEIGHT_MIN, WEIGHT_MIN,
                WEIGHT_MIN, WEIGHT_MIN,
                WEIGHT_MIN, WEIGHT_MIN,
                WEIGHT_MIN, WEIGHT_MIN);
        }
    }
}

} // namespace srs
